#!/usr/bin/env python3
"""
Hybrid ExG + YOLO weed detection.

Adds an Excess Green (ExG) colour-based pre-filter on top of the YOLO pipeline:

  - ExG (2G - R - B) runs on every frame on CPU (~1 ms) to find green blobs.
  - YOLO inference is skipped on frames where ExG finds nothing, saving GPU
    compute — useful on pavement/slate where most frames have no weeds.
  - ExG-only candidates are drawn in orange.
  - YOLO-confirmed tracks are drawn in purple (yellow when targeted).
  - Press 'e' to toggle a semi-transparent green ExG mask overlay.

Tune EXG_THRESH and EXG_MIN_AREA in .env for your surface:
  - Lower EXG_THRESH  → more sensitive, more false positives on concrete
  - Higher EXG_MIN_AREA → ignores tiny specks, catches only meaningful blobs
"""

import os
import threading
import time

import cv2
import numpy as np

import config
from dashboard import draw_stats_panel, _handle_overlay_click
from mavlink import _HAVE_MAVLINK, _mav_listener, _target_sock
from renderer import FrameGrabber, _Renderer, _make_no_stream_frame
from tracker import WeedTracker


def main() -> None:
    # Screen dimensions — AppKit gives the full frame including Dock area, which
    # matches OpenCV's fullscreen window exactly. tkinter can return the usable
    # area (Dock excluded) when the Dock is on the left or right, causing a gap.
    # CGDisplayBounds returns the full display frame in logical points — including the
    # Dock area — matching OpenCV's fullscreen window coordinate space exactly.
    # tkinter.winfo_screenwidth/height() subtracts a side-positioned Dock, causing
    # the combined image to be narrower than the OpenCV window and leaving a visible gap.
    screen_w, screen_h = 2560, 1440  # safe fallback
    try:
        import ctypes
        _cg = ctypes.CDLL('/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics')

        class _CGPoint(ctypes.Structure):
            _fields_ = [('x', ctypes.c_double), ('y', ctypes.c_double)]
        class _CGSize(ctypes.Structure):
            _fields_ = [('width', ctypes.c_double), ('height', ctypes.c_double)]
        class _CGRect(ctypes.Structure):
            _fields_ = [('origin', _CGPoint), ('size', _CGSize)]

        _cg.CGMainDisplayID.restype  = ctypes.c_uint32
        _cg.CGDisplayBounds.restype  = _CGRect
        _cg.CGDisplayBounds.argtypes = [ctypes.c_uint32]
        _b = _cg.CGDisplayBounds(_cg.CGMainDisplayID())
        screen_w = int(_b.size.width)
        screen_h = int(_b.size.height)
    except Exception:
        try:
            import tkinter as _tk
            _root = _tk.Tk(); _root.withdraw()
            screen_w, screen_h = _root.winfo_screenwidth(), _root.winfo_screenheight()
            _root.destroy()
        except Exception:
            pass
    print(f"Screen: {screen_w}×{screen_h}", flush=True)

    print(f"ExG threshold: {config.EXG_THRESH}  min blob: {config.EXG_MIN_AREA}px²", flush=True)
    print("Keys: 'e' ExG overlay  'q' quit  |  click blob to name (W1,W2…), click name to remove",
          flush=True)

    if _HAVE_MAVLINK:
        threading.Thread(target=_mav_listener, daemon=True).start()
    else:
        print("pymavlink not installed — world-frame re-ID disabled (pip install pymavlink)",
              flush=True)

    # Combined image must be exactly screen_w × screen_h so it fills the display.
    # Decide STATS_W first, then give the remainder to the video panel.
    frame_w, frame_h = 1920, 1080   # defaults; updated when stream connects
    config.STATS_W = max(screen_w // 2, 800)
    disp_w         = max(400, screen_w - config.SIDEBAR_W - config.STATS_W) + 50
    config.STATS_W = screen_w - config.SIDEBAR_W - disp_w   # recalculate after flooring disp_w
    disp_h         = screen_h + 220                          # Hardcoded for now
    disp_scale     = disp_h / frame_h

    # Standby overlay image
    _no_stream_img = None
    _nsi_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets', 'feednotfound.png')
    if os.path.exists(_nsi_path):
        _no_stream_img = cv2.imread(_nsi_path, cv2.IMREAD_UNCHANGED)

    tracker   = WeedTracker()
    tracker.set_frame_size(frame_w, frame_h)
    _btn_rect = [None]   # (x, y, w, h) of Connect Device btn in display coords

    # _conn tracks background stream-open state
    _conn = {'cap': None, 'ok': False, 'busy': False}

    def _open_stream():
        for backend, url in [(cv2.CAP_GSTREAMER, config.GST_PIPELINE),
                              (cv2.CAP_FFMPEG, config.FFMPEG_URL)]:
            c = cv2.VideoCapture(url, backend)
            if c.isOpened():
                _conn['cap'] = c
                _conn['ok']  = True
                break
            c.release()
        _conn['busy'] = False

    # Kick off initial connection attempt in background — window opens immediately
    _conn['busy'] = True
    threading.Thread(target=_open_stream, daemon=True).start()

    WIN = "Weed Detection — ExG"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    cap       = None
    stream_ok = False

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not stream_ok:
            # Trigger reconnect only when not already connecting
            if not _conn['busy']:
                br = _btn_rect[0]
                if br and br[0] <= x <= br[0] + br[2] and br[1] <= y <= br[1] + br[3]:
                    _conn.update({'cap': None, 'ok': False, 'busy': True})
                    threading.Thread(target=_open_stream, daemon=True).start()
            return
        if x < disp_w:
            tracker.queue_click(int(x / disp_scale), int(y / disp_scale))
        elif x < disp_w + config.SIDEBAR_W:
            tracker.sidebar_click(y)
        else:
            px = x - (disp_w + config.SIDEBAR_W)
            COL = config.STATS_W // 2
            if px >= COL and y >= disp_h - config.OVERLAY_H:
                _handle_overlay_click(px - COL, y - (disp_h - config.OVERLAY_H))

    cv2.setMouseCallback(WIN, on_mouse)

    # Show standby while the macOS fullscreen animation completes, then probe the
    # actual window content size from OpenCV directly before entering the main loop.
    _probe_end = time.time() + 0.75
    while time.time() < _probe_end:
        _pd, _pb = _make_no_stream_frame(disp_w, disp_h, _no_stream_img)
        _btn_rect[0] = _pb
        _probe_combined = np.hstack([_pd, tracker.draw_sidebar(disp_h), draw_stats_panel(disp_h)])
        cv2.imshow(WIN, _probe_combined)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            return

    # ── Pass 1: find true window WIDTH ──────────────────────────────────────────
    # Send a content-filled image much wider than the window. In KEEPRATIO mode
    # the width becomes the bottleneck and getWindowImageRect[2] = window width.
    _oversize = np.zeros((disp_h, screen_w + 400, 3), dtype=np.uint8)
    _oversize[:, :_probe_combined.shape[1]] = _probe_combined
    cv2.imshow(WIN, _oversize)
    cv2.waitKey(80)
    _wr = cv2.getWindowImageRect(WIN)
    _actual_w = _wr[2]
    if 100 < _actual_w < screen_w:  # window can only be narrower than the display, never wider
        screen_w       = _actual_w
        config.STATS_W = max(screen_w // 2, 800)
        disp_w         = max(400, screen_w - config.SIDEBAR_W - config.STATS_W)
        config.STATS_W = screen_w - config.SIDEBAR_W - disp_w

    # ── Pass 2: find true window HEIGHT from y-offset ────────────────────────
    # Show the correctly-sized image. In KEEPRATIO + centred mode the image
    # is padded top/bottom so: window_h = rendered_h + 2 * y_offset.
    _pd2, _pb2 = _make_no_stream_frame(disp_w, disp_h, _no_stream_img)
    _sized = np.hstack([_pd2, tracker.draw_sidebar(disp_h), draw_stats_panel(disp_h)])
    cv2.imshow(WIN, _sized)
    cv2.waitKey(80)
    _wr2      = cv2.getWindowImageRect(WIN)
    _actual_h = _wr2[3] + 2 * _wr2[1]   # rendered_h + 2 × y_offset
    if _actual_h < 100:                  # sanity: discard clearly invalid probe data
        _actual_h = screen_h
    print(f"Window probe: {_actual_w}×{_actual_h} "
          f"(detected {screen_w}×{screen_h}, y_off={_wr2[1]})", flush=True)
    _btn_rect[0] = _pb2

    if _actual_w != screen_w or _actual_h != screen_h:
        screen_w, screen_h = _actual_w, _actual_h
        config.STATS_W = max(screen_w // 2, 800)
        disp_w         = max(400, screen_w - config.SIDEBAR_W - config.STATS_W)
        config.STATS_W = screen_w - config.SIDEBAR_W - disp_w
        disp_h         = screen_h
        disp_scale     = disp_h / frame_h
        print(f"Layout corrected to {screen_w}×{screen_h}", flush=True)

    renderer = _Renderer(tracker, disp_w, disp_h, _no_stream_img, _btn_rect)

    while True:
        # ── Stream connect ─────────────────────────────────────────────────────
        if not stream_ok and _conn['ok']:
            cap = _conn['cap']
            _fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            _fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if _fw > 0 and _fh > 0:
                frame_w, frame_h = _fw, _fh
                disp_scale = disp_h / frame_h
                tracker.set_frame_size(frame_w, frame_h)
            renderer.attach_grabber(FrameGrabber(cap))
            stream_ok = True
            print(f"Stream connected: {frame_w}×{frame_h}", flush=True)

        # ── Stream disconnect (detected by renderer) ───────────────────────────
        if stream_ok and not renderer.stream_alive:
            stream_ok = False
            cap = None
            _conn.update({'cap': None, 'ok': False, 'busy': False})
            print("Stream lost — showing standby display", flush=True)

        # ── Display + event handling ───────────────────────────────────────────
        to_show = renderer.latest_frame()
        if to_show is not None:
            cv2.imshow(WIN, to_show)
        key = cv2.waitKey(8) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e') and stream_ok:
            renderer.show_exg = not renderer.show_exg
            print(f"ExG overlay {'on' if renderer.show_exg else 'off'}", flush=True)

    renderer.release()
    cv2.destroyAllWindows()
    _target_sock.close()
    print(f"\nDone. {renderer.frame_count} frames processed.", flush=True)


if __name__ == '__main__':
    main()
