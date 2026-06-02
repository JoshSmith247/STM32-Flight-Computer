"""
YOLOv8-based person tracker for Follow Me mode.

Inference runs in a background thread that communicates with _yolo_worker.py
via stdin/stdout pipes.  process_frame() is non-blocking: it enqueues the frame
and returns the last available detections immediately so the renderer thread
is never stalled waiting for YOLO.

The worker is a separate subprocess so its Metal context is completely isolated
from DearPyGUI's Metal render thread — this prevents the SIGSEGV that occurs
when PyTorch and DPG share Metal across threads on macOS ARM-64.

Usage:
  tracker = PersonTracker()
  tracker.set_frame_size(w, h)
  tracker.preload()                   # optional: warm up worker in background

  # Per-frame, in the renderer thread (non-blocking):
  detections = tracker.process_frame(frame)
  tracker.process_click(detections)
  tracker.update(detections)
  display = tracker.draw(frame, detections)

  wp = tracker.get_world_pos()        # (east_m, north_m) or None

  tracker.shutdown()                  # stop inference thread + worker on exit
"""

import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

import config
import gfx
from mavlink import pixel_to_world

_IOU_THRESH      = 0.35
_MAX_LOST        = 45
_VEL_HISTORY     = 6
_VEL_CAP_MS      = 5.0
_STALE_TIMEOUT_S = 2.0

_WORKER_SCRIPT = os.path.join(os.path.dirname(__file__), '_yolo_worker.py')


def _iou(a: tuple, b: tuple) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter  = max(0, x2 - x1) * max(0, y2 - y1)
    if inter == 0:
        return 0.0
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter)


class PersonTracker:
    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._proc_lock = threading.Lock()

        # Inference runs in a background thread; frames are queued here.
        self._frame_q: queue.Queue = queue.Queue(maxsize=1)
        self._det_lock  = threading.Lock()
        self._detections: list = []

        self._tracked: tuple | None = None
        self._lost_frames    = 0
        self._frame_size: tuple | None = None
        self._pending_click: tuple | None = None

        self._pos_history: list = []
        self._world_vel: tuple  = (0.0, 0.0)
        self._pixel_vel: tuple  = (0.0, 0.0)
        self._last_lock_t: float = 0.0

        self._stop_infer = False
        self._infer_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name='yolo-infer')
        self._infer_thread.start()

    def set_frame_size(self, w: int, h: int) -> None:
        self._frame_size = (w, h)

    # ── Worker subprocess ──────────────────────────────────────────────────────

    def _start_worker(self) -> None:
        """Spawn the YOLO subprocess and block until it signals ready.
        Does NOT hold _proc_lock while blocking so shutdown() can proceed."""
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                return  # already live

        proc = subprocess.Popen(
            [sys.executable, _WORKER_SCRIPT, config.YOLO_MODEL],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,   # pass YOLO download progress to terminal
        )
        # Store proc immediately so shutdown() can kill it even while we are
        # blocked on readline() waiting for the worker to finish loading.
        with self._proc_lock:
            if self._proc is not None and self._proc.poll() is None:
                proc.kill()   # lost the race — another thread won
                return
            self._proc = proc

        # Block here (no lock held) — may take tens of seconds if downloading.
        line = proc.stdout.readline().decode().strip()
        if not line:
            return  # pipe closed — parent is shutting down
        if line != 'ready':
            proc.kill()
            raise RuntimeError(f"YOLO worker failed to init: {line!r}")

        print("YOLO worker ready  (subprocess, isolated Metal context)", flush=True)

    def preload(self) -> None:
        """Start the worker in a background thread so it is warm before first use."""
        threading.Thread(target=self._start_worker, daemon=True,
                         name='yolo-preload').start()

    # Alias for any existing callers.
    _load_model = preload

    def shutdown(self) -> None:
        """Stop the inference thread and worker subprocess."""
        self._stop_infer = True
        # Unblock the inference thread if it is waiting on the queue.
        try:
            self._frame_q.put_nowait(None)
        except queue.Full:
            pass
        self._infer_thread.join(timeout=3.0)
        with self._proc_lock:
            if self._proc is not None:
                try:
                    self._proc.stdin.close()
                    self._proc.wait(timeout=2.0)
                except Exception:
                    self._proc.kill()
                self._proc = None

    # ── Background inference loop ──────────────────────────────────────────────

    def _inference_loop(self) -> None:
        while not self._stop_infer:
            try:
                frame = self._frame_q.get(timeout=0.2)
            except queue.Empty:
                continue

            if frame is None:   # shutdown sentinel
                break

            # Ensure worker is alive (start or restart if needed).
            with self._proc_lock:
                proc = self._proc
            if proc is None or proc.poll() is not None:
                try:
                    self._start_worker()
                except Exception as exc:
                    print(f"[follow] worker start failed: {exc}", flush=True)
                    continue
                with self._proc_lock:
                    proc = self._proc
                if proc is None:
                    continue

            h, w, c = frame.shape
            header = struct.pack('<iii', h, w, c)
            try:
                proc.stdin.write(header + frame.tobytes())
                proc.stdin.flush()
                line = proc.stdout.readline()
                if not line:
                    raise BrokenPipeError("worker stdout closed")
                raw = json.loads(line)
                boxes = [(x1, y1, x2, y2, conf) for x1, y1, x2, y2, conf in raw]
                with self._det_lock:
                    self._detections = boxes
            except Exception as exc:
                print(f"[follow] worker error: {exc} — restarting next frame", flush=True)
                with self._proc_lock:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
                    self._proc = None

    # ── Per-frame API (non-blocking) ──────────────────────────────────────────

    def process_frame(self, frame: np.ndarray) -> list:
        """Queue frame for async YOLO inference; return last available detections.

        Never blocks — if the inference thread is busy the old frame is dropped
        and the latest is queued instead.
        """
        try:
            self._frame_q.put_nowait(frame)
        except queue.Full:
            try:
                self._frame_q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._frame_q.put_nowait(frame)
            except queue.Full:
                pass

        with self._det_lock:
            return list(self._detections)

    def update(self, detections: list) -> None:
        """Advance tracking one frame: match tracked box to best IoU detection."""
        if self._tracked is None:
            return
        best_iou, best = 0.0, None
        for x1, y1, x2, y2, _ in detections:
            iou = _iou(self._tracked, (x1, y1, x2, y2))
            if iou > best_iou:
                best_iou, best = iou, (x1, y1, x2, y2)
        if best_iou >= _IOU_THRESH:
            old_cx = (self._tracked[0] + self._tracked[2]) / 2
            old_cy = (self._tracked[1] + self._tracked[3]) / 2
            new_cx = (best[0] + best[2]) / 2
            new_cy = (best[1] + best[3]) / 2
            dx, dy = new_cx - old_cx, new_cy - old_cy
            pvx = 0.6 * self._pixel_vel[0] + 0.4 * dx
            pvy = 0.6 * self._pixel_vel[1] + 0.4 * dy
            self._pixel_vel = (pvx, pvy)

            self._tracked     = best
            self._lost_frames = 0
            self._last_lock_t = time.monotonic()

            wp = self.get_world_pos()
            if wp is not None:
                self._update_velocity(wp)
        else:
            self._pixel_vel   = (self._pixel_vel[0] * 0.5, self._pixel_vel[1] * 0.5)
            self._lost_frames += 1
            if self._lost_frames > _MAX_LOST:
                self._tracked     = None
                self._lost_frames = 0
                self._world_vel   = (0.0, 0.0)
                self._pixel_vel   = (0.0, 0.0)
                self._pos_history.clear()
                print("Follow: target lost — deselected", flush=True)

    # ── Click handling ────────────────────────────────────────────────────────

    def queue_click(self, x: int, y: int) -> None:
        self._pending_click = (x, y)

    def process_click(self, detections: list) -> None:
        if self._pending_click is None:
            return
        x, y = self._pending_click
        self._pending_click = None

        if self._tracked is not None:
            x1, y1, x2, y2 = self._tracked
            if x1 <= x <= x2 and y1 <= y <= y2:
                self._tracked     = None
                self._lost_frames = 0
                print("Follow: deselected", flush=True)
                return

        for x1, y1, x2, y2, _ in detections:
            if x1 <= x <= x2 and y1 <= y <= y2:
                self._tracked     = (x1, y1, x2, y2)
                self._lost_frames = 0
                self._world_vel   = (0.0, 0.0)
                self._pixel_vel   = (0.0, 0.0)
                self._pos_history.clear()
                self._last_lock_t = time.monotonic()
                wp  = self.get_world_pos()
                loc = f"{wp[0]:.2f}m E, {wp[1]:.2f}m N" if wp else "no GPS fix"
                print(f"Follow: target acquired  ({loc})", flush=True)
                return

    # ── World position & velocity ─────────────────────────────────────────────

    def get_world_pos(self) -> tuple[float, float] | None:
        if self._tracked is None or self._frame_size is None:
            return None
        x1, y1, x2, y2 = self._tracked
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        return pixel_to_world(cx, cy, self._frame_size[0], self._frame_size[1])

    def get_follow_target(self, predict_s: float = 0.0) -> tuple[float, float] | None:
        wp = self.get_world_pos()
        if wp is None:
            return None
        if predict_s <= 0.0:
            return wp
        ve, vn = self._world_vel
        speed = math.hypot(ve, vn)
        if speed > _VEL_CAP_MS:
            scale = _VEL_CAP_MS / speed
            ve, vn = ve * scale, vn * scale
        return (wp[0] + ve * predict_s, wp[1] + vn * predict_s)

    def _update_velocity(self, wp: tuple) -> None:
        now = time.monotonic()
        self._pos_history.append((now, wp[0], wp[1]))
        if len(self._pos_history) > _VEL_HISTORY:
            self._pos_history.pop(0)
        if len(self._pos_history) < 2:
            return
        dt = self._pos_history[-1][0] - self._pos_history[0][0]
        if dt < 0.1:
            return
        de = self._pos_history[-1][1] - self._pos_history[0][1]
        dn = self._pos_history[-1][2] - self._pos_history[0][2]
        self._world_vel = (de / dt, dn / dt)

    @property
    def ground_speed_ms(self) -> float:
        return math.hypot(*self._world_vel)

    # ── Rendering ─────────────────────────────────────────────────────────────

    def draw(self, frame: np.ndarray, detections: list) -> np.ndarray:
        out = frame.copy()
        spd = self.ground_speed_ms
        for x1, y1, x2, y2, conf in detections:
            is_tracked = (self._tracked is not None and
                          (x1, y1, x2, y2) == self._tracked)
            if is_tracked:
                active = self._lost_frames == 0
                stale  = (time.monotonic() - self._last_lock_t) > _STALE_TIMEOUT_S
                if active:
                    color = (147, 20, 255)
                elif stale:
                    color = (80, 80, 100)
                else:
                    color = (130, 80, 160)
                thick = 3
                if active:
                    label = f'FOLLOWING  {spd:.1f}m/s' if spd > 0.15 else 'FOLLOWING'
                elif stale:
                    label = 'STALE HOLD'
                else:
                    label = f'SEARCHING {self._lost_frames}'
            else:
                color = (203, 192, 255)
                thick = 2
                label = f'{conf:.0%}'
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
            cv2.putText(out, label, (x1 + 3, y1 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

            if is_tracked and self._lost_frames == 0:
                pvx, pvy = self._pixel_vel
                pspd = math.hypot(pvx, pvy)
                if pspd > 1.5:
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    arrow_len = int(min(60, max(20, pspd * 8)))
                    ex = int(cx + pvx / pspd * arrow_len)
                    ey = int(cy + pvy / pspd * arrow_len)
                    cv2.arrowedLine(out, (cx, cy), (ex, ey), color, 2, tipLength=0.35)

        return out

    def draw_sidebar(self, h: int) -> np.ndarray:
        s     = config.PR
        W     = config.SIDEBAR_W * s
        H     = h * s
        panel = np.full((H, W, 3), (35, 35, 35), dtype=np.uint8)
        gfx.begin()

        HDR = config.SIDEBAR_HDR_H * s
        cv2.rectangle(panel, (0, 0), (W, HDR - s), (40, 44, 50), -1)
        gfx.put_text(panel, "Follow Me", (8 * s, 23 * s), 0.5 * s, (185, 192, 200))
        cv2.line(panel, (0, HDR - s), (W, HDR - s), (60, 60, 60), max(1, s))

        ry = HDR + 14 * s
        if self._tracked is not None:
            active = self._lost_frames == 0
            stale  = (time.monotonic() - self._last_lock_t) > _STALE_TIMEOUT_S
            if active:
                dot_col = (147, 20, 255)
                txt_col = (203, 192, 255)
                lbl     = 'FOLLOWING'
            elif stale:
                dot_col = (80, 80, 100)
                txt_col = (110, 100, 120)
                lbl     = 'STALE HOLD'
            else:
                dot_col = (130, 80, 160)
                txt_col = (140, 100, 170)
                lbl     = f'SEARCHING ({self._lost_frames})'
            cv2.circle(panel, (12 * s, ry), 4 * s, dot_col, -1)
            gfx.put_text(panel, lbl, (22 * s, ry + 5 * s), 0.42 * s, txt_col)
            if self.get_world_pos() is not None:
                gfx.put_text(panel, 'GPS', (W - 38 * s, ry + 5 * s),
                             0.35 * s, (80, 150, 80))

            spd = self.ground_speed_ms
            if spd > 0.15:
                spd_s = f"{spd:.1f} m/s"
                gfx.put_text(panel, spd_s, (22 * s, ry + 20 * s), 0.40 * s, txt_col)
        else:
            gfx.put_text(panel, 'Click person', (8 * s, ry + 5 * s),
                         0.38 * s, (90, 90, 90))

        gfx.flush(panel)
        return panel
