import os
import threading
import time

import cv2
import numpy as np

import config
from dashboard import draw_stats_panel
from exg import exg_candidates, exg_mask
from mavlink import send_weed_target
from tracker import WeedTracker

_LOGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')


class _VideoRecorder:
    """Thread-safe cv2.VideoWriter wrapper; lazy-opens on the first frame."""

    def __init__(self, logs_dir: str) -> None:
        self._logs_dir = logs_dir
        self._writer: cv2.VideoWriter | None = None
        self._path: str | None = None
        self._lock = threading.Lock()

    def write(self, frame: np.ndarray) -> None:
        with self._lock:
            if self._writer is None:
                os.makedirs(self._logs_dir, exist_ok=True)
                h, w = frame.shape[:2]
                ts = time.strftime('%Y%m%d_%H%M%S')
                self._path = os.path.join(self._logs_dir, f'record_{ts}.mp4')
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self._writer = cv2.VideoWriter(self._path, fourcc, 30.0, (w, h))
                print(f"Recording → {self._path}", flush=True)
            self._writer.write(frame)

    def close(self) -> None:
        with self._lock:
            if self._writer is not None:
                self._writer.release()
                self._writer = None
                print(f"Recording saved: {self._path}", flush=True)


def _make_no_stream_frame(w: int, h: int, img) -> tuple:
    """Physical-pixel standby frame; returns logical btn_rect for click detection."""
    s = config.PR
    W, H = w * s, h * s
    frame    = np.full((H, W, 3), 72, dtype=np.uint8)
    btn_w_l  = 200          # logical button dimensions
    btn_h_l  = 42
    btn_w, btn_h = btn_w_l * s, btn_h_l * s

    if img is not None:
        ih, iw = img.shape[:2]
        max_iw = min(W * 2 // 3, W - 40 * s)
        max_ih = min(H * 2 // 5, H - 120 * s)
        sc     = min(max_iw / iw, max_ih / ih)
        niw, nih = int(iw * sc), int(ih * sc)
        resized = cv2.resize(img, (niw, nih), interpolation=cv2.INTER_AREA)
        if resized.ndim == 3 and resized.shape[2] == 4:
            alpha   = resized[:, :, 3:4].astype(np.float32) / 255.0
            bg      = np.full((nih, niw, 3), 72, dtype=np.float32)
            resized = (resized[:, :, :3].astype(np.float32) * alpha
                       + bg * (1.0 - alpha)).astype(np.uint8)
        ox  = (W - niw) // 2
        oy  = max(0, (H - nih) // 2 - btn_h - 20 * s)
        frame[oy:oy + nih, ox:ox + niw] = resized
        btn_y = min(oy + nih + 20 * s, H - btn_h - 10 * s)
    else:
        (tw, _), _ = cv2.getTextSize('NO STREAM', cv2.FONT_HERSHEY_SIMPLEX, 1.2 * s, 2)
        cv2.putText(frame, 'NO STREAM', (W // 2 - tw // 2, H // 2 - 20 * s),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2 * s, (130, 130, 130),
                    max(1, 2 * s), cv2.LINE_AA)
        btn_y = H // 2 + 30 * s

    btn_x = (W - btn_w) // 2
    cv2.rectangle(frame, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), (60, 110, 190), -1)
    cv2.rectangle(frame, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h),
                  (90, 145, 225), max(1, s))
    lbl = 'Connect Device'
    (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.52 * s, 1)
    cv2.putText(frame, lbl,
                (btn_x + (btn_w - lw) // 2, btn_y + (btn_h + lh) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52 * s, (230, 230, 230),
                max(1, s), cv2.LINE_AA)

    # Return logical btn_rect so station.py click detection works with logical mouse coords
    return frame, (btn_x // s, btn_y // s, btn_w_l, btn_h_l)


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
        self._vid_arr:     np.ndarray | None = None
        self._sidebar_arr: np.ndarray | None = None
        self._stats_arr:   np.ndarray | None = None
        self._vid_seq     = 0
        self._sidebar_seq = 0
        self._stats_seq   = 0
        self._lock     = threading.Lock()
        self.show_exg     = False
        self.frame_count  = 0
        self.stream_alive = False
        self._prev_sel: int | None = None
        # Letterbox state — written by renderer thread, read by main thread (GIL-safe)
        self._vid_scale = 1.0
        self._vid_x_off = 0
        self._vid_y_off = 0

        self._recorder: _VideoRecorder | None = None

        threading.Thread(target=self._run, daemon=True).start()

    def attach_grabber(self, grabber: FrameGrabber) -> None:
        with self._lock:
            self._grabber     = grabber
            self.stream_alive = True

    def display_to_frame(self, dx: int, dy: int) -> tuple[int, int] | None:
        """Map a logical-pixel display click to source frame coordinates.
        Returns None if the point lands in a letterbox border."""
        pr = config.PR
        fx = (dx * pr - self._vid_x_off) / self._vid_scale
        fy = (dy * pr - self._vid_y_off) / self._vid_scale
        fs = self._tracker._frame_size
        if fx < 0 or fy < 0 or (fs is not None and (fx >= fs[0] or fy >= fs[1])):
            return None
        return int(fx), int(fy)

    def latest_panels(self) -> tuple | None:
        with self._lock:
            if self._vid_arr is None:
                return None
            return (self._vid_arr, self._sidebar_arr, self._stats_arr,
                    self._vid_seq, self._sidebar_seq, self._stats_seq)

    def release(self) -> None:
        with self._lock:
            g = self._grabber
            self._grabber     = None
            self.stream_alive = False
        if g is not None:
            g.release()
        if self._recorder is not None:
            self._recorder.close()

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
                with self._lock:
                    self._sidebar_arr = _sidebar
                    self._sidebar_seq += 1

            now = time.monotonic()
            if _stats is None or now >= _stats_next_t:
                _stats       = draw_stats_panel(self._disp_h, self._tracker)
                _stats_next_t = now + _STATS_INTERVAL
                with self._lock:
                    self._stats_arr = _stats
                    self._stats_seq += 1

            # ── No active stream: build standby frame at ~30 fps ─────────────
            if grabber is None or not alive:
                display, btn = _make_no_stream_frame(
                    self._disp_w, self._disp_h, self._nsi)
                self._btn_out[0] = btn
                with self._lock:
                    self._vid_arr = display
                    self._vid_seq += 1
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

            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            mask  = exg_mask(frame)
            blobs = exg_candidates(mask)
            self._tracker.process_click(frame)
            self._tracker.propagate_named(frame, blobs)
            self._tracker.update_exg(blobs)
            display = self._tracker.draw(frame, blobs)

            if self.show_exg:
                tint = np.zeros_like(display)
                tint[:, :, 1] = mask
                display = cv2.addWeighted(display, 1.0, tint, 0.4, 0)

            # Start/stop recorder based on runtime flag
            want_rec = config.RECORD_ACTIVE
            if want_rec and self._recorder is None:
                self._recorder = _VideoRecorder(_LOGS_DIR)
            elif not want_rec and self._recorder is not None:
                self._recorder.close()
                self._recorder = None
            if self._recorder is not None:
                self._recorder.write(display)

            pr    = config.PR
            pw    = self._disp_w * pr   # physical canvas dimensions
            ph    = self._disp_h * pr
            src_h, src_w = display.shape[:2]
            scale  = min(pw / src_w, ph / src_h)
            fit_w  = int(src_w * scale)
            fit_h  = int(src_h * scale)
            x_off  = (pw - fit_w) // 2
            y_off  = (ph - fit_h) // 2
            resized = cv2.resize(display, (fit_w, fit_h), interpolation=cv2.INTER_LINEAR)
            canvas  = np.zeros((ph, pw, 3), dtype=np.uint8)
            canvas[y_off:y_off + fit_h, x_off:x_off + fit_w] = resized
            display = canvas
            self._vid_scale = scale   # physical px / source px
            self._vid_x_off = x_off   # physical px
            self._vid_y_off = y_off

            n_active = sum(1 for w in self._tracker._named.values()
                           if w['lost_frames'] == 0)
            n_lost   = len(self._tracker._named) - n_active
            hud = f"ExG blobs: {len(blobs)}"
            if n_active:
                hud += f"  |  named: {n_active}"
            if n_lost:
                hud += f"  |  searching: {n_lost}"
            cv2.putText(display, hud, (9 * pr, 23 * pr),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * pr, (0, 0, 0),
                        max(1, pr), cv2.LINE_AA)
            cv2.putText(display, hud, (8 * pr, 22 * pr),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * pr, (220, 220, 220),
                        max(1, pr), cv2.LINE_AA)

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

            with self._lock:
                self._vid_arr = display
                self._vid_seq += 1
            self.frame_count += 1
