import threading
import time

import cv2
import numpy as np

from dashboard import draw_stats_panel
from exg import exg_candidates, exg_mask
from mavlink import send_weed_target
from tracker import WeedTracker


def _make_no_stream_frame(w: int, h: int, img) -> tuple:
    """Gray standby frame with optional overlay image and a Connect Device button."""
    frame  = np.full((h, w, 3), 72, dtype=np.uint8)
    btn_w, btn_h = 200, 42

    if img is not None:
        ih, iw = img.shape[:2]
        max_iw = min(w * 2 // 3, w - 40)
        max_ih = min(h * 2 // 5, h - 120)
        scale  = min(max_iw / iw, max_ih / ih)
        niw, nih = int(iw * scale), int(ih * scale)
        resized = cv2.resize(img, (niw, nih), interpolation=cv2.INTER_AREA)
        if resized.ndim == 3 and resized.shape[2] == 4:
            alpha   = resized[:, :, 3:4].astype(np.float32) / 255.0
            bg      = np.full((nih, niw, 3), 72, dtype=np.float32)
            resized = (resized[:, :, :3].astype(np.float32) * alpha
                       + bg * (1.0 - alpha)).astype(np.uint8)
        ox  = (w - niw) // 2
        oy  = max(0, (h - nih) // 2 - btn_h - 20)
        frame[oy:oy + nih, ox:ox + niw] = resized
        btn_y = min(oy + nih + 20, h - btn_h - 10)
    else:
        cv2.putText(frame, 'NO STREAM', (w // 2 - 90, h // 2 - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (130, 130, 130), 2, cv2.LINE_AA)
        btn_y = h // 2 + 30

    btn_x = (w - btn_w) // 2
    cv2.rectangle(frame, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (60, 110, 190), -1)
    cv2.rectangle(frame, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (90, 145, 225), 1)
    lbl = 'Connect Device'
    (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
    cv2.putText(frame, lbl, (btn_x + (btn_w - lw) // 2, btn_y + (btn_h + lh) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (230, 230, 230), 1, cv2.LINE_AA)

    return frame, (btn_x, btn_y, btn_w, btn_h)


class FrameGrabber:
    """Drains cap.read() in a background thread so the main loop never blocks on decode.

    The thread runs as fast as the decoder allows, always keeping only the
    latest decoded frame.  Main loop calls read() to get it without waiting.
    """

    def __init__(self, cap: cv2.VideoCapture) -> None:
        self._cap   = cap
        self._frame: np.ndarray | None = None
        self._ok    = True
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        while True:
            ok, frame = self._cap.read()
            if not ok:
                with self._lock:
                    self._ok = False
                break
            with self._lock:
                self._frame = frame
        self._cap.release()

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Non-blocking.  Returns (stream_alive, latest_frame_or_None)."""
        with self._lock:
            return self._ok, self._frame

    def release(self) -> None:
        """Interrupt the grabber by releasing the underlying capture device."""
        self._cap.release()


class _Renderer:
    """Runs the full ExG + tracking + drawing pipeline in a background thread.

    Produces a ready-to-display combined numpy frame.  The main loop only
    calls imshow() + waitKey(8), so mouse/keyboard events are serviced every
    ~8 ms regardless of how long a frame takes to process.
    """

    def __init__(self, tracker: WeedTracker, disp_w: int, disp_h: int,
                 no_stream_img, btn_rect_out: list) -> None:
        self._tracker  = tracker
        self._disp_w   = disp_w
        self._disp_h   = disp_h
        self._nsi      = no_stream_img
        self._btn_out  = btn_rect_out

        self._grabber: FrameGrabber | None = None
        self._combined: np.ndarray | None  = None
        self._lock     = threading.Lock()
        self.show_exg     = False
        self.frame_count  = 0
        self.stream_alive = False
        self._prev_sel: int | None = None

        threading.Thread(target=self._run, daemon=True).start()

    def attach_grabber(self, grabber: FrameGrabber) -> None:
        with self._lock:
            self._grabber     = grabber
            self.stream_alive = True

    def latest_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._combined

    def release(self) -> None:
        with self._lock:
            g = self._grabber
            self._grabber     = None
            self.stream_alive = False
        if g is not None:
            g.release()

    def _run(self) -> None:
        # Cached panels — rebuilt only when content changes, not every frame.
        _sidebar:       np.ndarray | None = None
        _sidebar_key    = (-1, None)          # (n_weeds, selected_wid)
        _stats:         np.ndarray | None = None
        _stats_next_t   = 0.0                 # next time to redraw stats (10 fps cap)
        _STATS_INTERVAL = 0.10                # seconds between stats redraws

        while True:
            with self._lock:
                grabber = self._grabber
                alive   = self.stream_alive

            # ── Rebuild cached panels only when needed ────────────────────────
            sidebar_key = (len(self._tracker._named), self._tracker._selected_wid)
            if _sidebar is None or sidebar_key != _sidebar_key:
                _sidebar     = self._tracker.draw_sidebar(self._disp_h)
                _sidebar_key = sidebar_key

            now = time.monotonic()
            if _stats is None or now >= _stats_next_t:
                _stats       = draw_stats_panel(self._disp_h)
                _stats_next_t = now + _STATS_INTERVAL

            # ── No active stream: build standby frame at ~30 fps ─────────────
            if grabber is None or not alive:
                display, btn = _make_no_stream_frame(
                    self._disp_w, self._disp_h, self._nsi)
                self._btn_out[0] = btn
                combined = np.hstack([display, _sidebar, _stats])
                with self._lock:
                    self._combined = combined
                time.sleep(1 / 30)
                continue

            # ── Active stream ─────────────────────────────────────────────────
            ok, frame = grabber.read()
            if not ok:
                with self._lock:
                    self.stream_alive = False
                    self._grabber     = None
                continue
            if frame is None:
                time.sleep(0.005)
                continue

            mask  = exg_mask(frame)
            blobs = exg_candidates(mask)
            self._tracker.process_click(frame)
            self._tracker.propagate_named(frame, blobs)
            self._tracker.propagate(frame)
            self._tracker.update_exg(blobs)
            display = self._tracker.draw(frame, blobs)

            if self.show_exg:
                tint = np.zeros_like(display)
                tint[:, :, 1] = mask
                display = cv2.addWeighted(display, 1.0, tint, 0.4, 0)

            display = cv2.resize(display, (self._disp_w, self._disp_h),
                                 interpolation=cv2.INTER_LINEAR)

            n_active = sum(1 for w in self._tracker._named.values()
                           if w['lost_frames'] == 0)
            n_lost   = len(self._tracker._named) - n_active
            hud = f"ExG blobs: {len(blobs)}"
            if n_active:
                hud += f"  |  named: {n_active}"
            if n_lost:
                hud += f"  |  searching: {n_lost}"
            cv2.putText(display, hud, (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

            sel = self._tracker._selected_wid
            if sel != self._prev_sel:
                self._prev_sel = sel
                if sel is not None:
                    wp = self._tracker._named.get(sel, {}).get('world_pos')
                    if wp is not None:
                        send_weed_target(sel, wp[0], wp[1])
                    else:
                        print(f"W{sel} selected — no GPS fix yet, target not sent",
                              flush=True)

            # hstack outside the lock — only the pointer swap needs the lock
            combined = np.hstack([display, _sidebar, _stats])
            with self._lock:
                self._combined = combined
            self.frame_count += 1


# ---------------------------------------------------------------------------
# Background YOLO inference thread (commented out — ExG-only mode)
# ---------------------------------------------------------------------------

# from ultralytics import YOLO
# import queue
# model = YOLO(MODEL_PATH)
# _infer_queue: queue.Queue = queue.Queue(maxsize=1)
# _det_lock = threading.Lock()
# _latest_det = None

# def _inference_worker() -> None:
#     global _latest_det
#     while True:
#         frame = _infer_queue.get()
#         if frame is None:
#             break
#         results = model.track(
#             frame, conf=CONF_THRESH, iou=IOU_THRESH, persist=True,
#             verbose=False, imgsz=INFER_IMGSZ, device=DEVICE,
#         )
#         with _det_lock:
#             _latest_det = (frame, results[0].boxes, results[0].names)
