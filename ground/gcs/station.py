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
import dearpygui.dearpygui as dpg
import numpy as np

import config
from dashboard import _handle_overlay_click, handle_payload_double_click
from mavlink import _HAVE_MAVLINK, _mav_listener, _target_sock
from renderer import FrameGrabber, _Renderer
from tracker import WeedTracker


def _macos_set_presentation(hide: bool) -> None:
    """Hide or restore the macOS menu bar and Dock via the ObjC runtime.

    Uses NSApplication.setPresentationOptions: with HideMenuBar | HideDock.
    Safe to call on non-macOS (the import will silently fail).
    """
    try:
        import ctypes, ctypes.util
        libobjc = ctypes.cdll.LoadLibrary(ctypes.util.find_library('objc'))
        libobjc.objc_getClass.restype     = ctypes.c_void_p
        libobjc.objc_getClass.argtypes    = [ctypes.c_char_p]
        libobjc.sel_registerName.restype  = ctypes.c_void_p
        libobjc.sel_registerName.argtypes = [ctypes.c_char_p]

        def _msg0(obj, sel):
            f = libobjc.objc_msgSend
            f.restype  = ctypes.c_void_p
            f.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
            return f(obj, sel)

        def _msg_uint(obj, sel, val):
            f = libobjc.objc_msgSend
            f.restype  = None
            f.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint64]
            f(obj, sel, val)

        ns_app = _msg0(
            libobjc.objc_getClass(b'NSApplication'),
            libobjc.sel_registerName(b'sharedApplication'),
        )
        # NSApplicationPresentationAutoHideDock = 1, NSApplicationPresentationAutoHideMenuBar = 4
        # Auto-hide lets the user slide them back in by moving the cursor to the edge.
        _msg_uint(
            ns_app,
            libobjc.sel_registerName(b'setPresentationOptions:'),
            ctypes.c_uint64(1 | 4 if hide else 0),
        )
    except Exception as exc:
        print(f"macOS presentation: {exc}", flush=True)


def main() -> None:
    # ── Screen size ───────────────────────────────────────────────────────────
    # CGDisplayBounds covers the full display frame in logical points (including
    # the Dock area), matching DPG's viewport coordinate space exactly.
    screen_w, screen_h = 2560, 1440
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

    # ── Layout ────────────────────────────────────────────────────────────────
    frame_w, frame_h = 1920, 1080
    config.STATS_W = max(screen_w // 2, 800)
    disp_w         = max(400, screen_w - config.SIDEBAR_W - config.STATS_W) - 30
    config.STATS_W = screen_w - config.SIDEBAR_W - disp_w
    disp_h         = screen_h

    _no_stream_img = None
    _nsi_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             'assets', 'feednotfound.png')
    if os.path.exists(_nsi_path):
        _no_stream_img = cv2.imread(_nsi_path, cv2.IMREAD_UNCHANGED)

    tracker   = WeedTracker()
    tracker.set_frame_size(frame_w, frame_h)
    _btn_rect = [None]
    _conn     = {'cap': None, 'ok': False, 'busy': False}

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

    _conn['busy'] = True
    threading.Thread(target=_open_stream, daemon=True).start()

    # ── Dear PyGui setup ──────────────────────────────────────────────────────
    dpg.create_context()

    with dpg.texture_registry():
        dpg.add_raw_texture(disp_w, disp_h,
                            np.zeros(disp_w * disp_h * 4, dtype=np.float32),
                            format=dpg.mvFormat_Float_rgba, tag="tex_vid")
        dpg.add_raw_texture(config.SIDEBAR_W, disp_h,
                            np.zeros(config.SIDEBAR_W * disp_h * 4, dtype=np.float32),
                            format=dpg.mvFormat_Float_rgba, tag="tex_side")
        dpg.add_raw_texture(config.STATS_W, disp_h,
                            np.zeros(config.STATS_W * disp_h * 4, dtype=np.float32),
                            format=dpg.mvFormat_Float_rgba, tag="tex_stats")

    stream_ok       = [False]
    renderer        = [None]
    _last_click     = [0.0, -1, -1]   # [time, x, y] for double-click detection
    _DCLICK_S       = 0.40

    def _on_click(sender, app_data):
        now = time.monotonic()
        x, y = dpg.get_mouse_pos(local=False)
        x, y = int(x), int(y)
        is_double = (now - _last_click[0] < _DCLICK_S and
                     abs(x - _last_click[1]) < 20 and
                     abs(y - _last_click[2]) < 20)
        _last_click[:] = [now, x, y]
        if not stream_ok[0]:
            if not _conn['busy']:
                br = _btn_rect[0]
                if br and br[0] <= x <= br[0] + br[2] and br[1] <= y <= br[1] + br[3]:
                    _conn.update({'cap': None, 'ok': False, 'busy': True})
                    threading.Thread(target=_open_stream, daemon=True).start()
            return
        if x < disp_w:
            fc = renderer[0].display_to_frame(x, y) if renderer[0] else None
            if fc is not None:
                tracker.queue_click(*fc)
        elif x < disp_w + config.SIDEBAR_W:
            tracker.sidebar_click(y)
        else:
            px  = x - (disp_w + config.SIDEBAR_W)
            COL = config.STATS_W // 2
            if is_double and px >= COL:
                handle_payload_double_click(px, y)
            if px >= COL and y >= disp_h - config.OVERLAY_H:
                _handle_overlay_click(px - COL, y - (disp_h - config.OVERLAY_H))

    _quit = [False]

    def _on_key_interrupt(*_):
        _quit[0] = True

    def _on_key_e(*_):
        r = renderer[0]
        if r and stream_ok[0]:
            r.show_exg = not r.show_exg
            print(f"ExG overlay {'on' if r.show_exg else 'off'}", flush=True)

    with dpg.handler_registry():
        dpg.add_mouse_click_handler(button=dpg.mvMouseButton_Left, callback=_on_click)
        dpg.add_key_press_handler(key=dpg.mvKey_Q, callback=_on_key_interrupt)
        dpg.add_key_press_handler(key=dpg.mvKey_Escape, callback=_on_key_interrupt)
        dpg.add_key_press_handler(key=dpg.mvKey_E, callback=_on_key_e)

    sid_x   = disp_w
    stats_x = disp_w + config.SIDEBAR_W

    # viewport_drawlist renders directly onto the viewport surface — no window
    # chrome, no padding, and it always fills the viewport exactly.
    with dpg.viewport_drawlist(front=False, tag="canvas"):
        dpg.draw_image("tex_vid",   (0,       0), (disp_w,   screen_h))
        dpg.draw_image("tex_side",  (sid_x,   0), (stats_x,  screen_h))
        dpg.draw_image("tex_stats", (stats_x, 0), (screen_w, screen_h))

    dpg.create_viewport(title="Ground Control Station",
                        width=screen_w, height=screen_h,
                        x_pos=0, y_pos=0, decorated=False, resizable=False)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    _macos_set_presentation(True)   # hide menu bar + Dock

    # BGR uint8 → flat RGBA float32 for DPG texture upload
    def _to_tex(bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        out = np.empty((h, w, 4), dtype=np.float32)
        out[:, :, 0] = bgr[:, :, 2] * (1.0 / 255)
        out[:, :, 1] = bgr[:, :, 1] * (1.0 / 255)
        out[:, :, 2] = bgr[:, :, 0] * (1.0 / 255)
        out[:, :, 3] = 1.0
        return out.ravel()

    renderer[0] = _Renderer(tracker, disp_w, disp_h, _no_stream_img, _btn_rect)
    cap         = None
    _seen       = [-1, -1, -1]   # last uploaded [vid_seq, sidebar_seq, stats_seq]

    while dpg.is_dearpygui_running() and not _quit[0]:
        r = renderer[0]

        # ── Stream connect ─────────────────────────────────────────────────────
        if not stream_ok[0] and _conn['ok']:
            cap = _conn['cap']
            _fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            _fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if _fw > 0 and _fh > 0:
                tracker.set_frame_size(_fh, _fw)  # swapped: camera rotated 90°
            r.attach_grabber(FrameGrabber(cap))
            stream_ok[0] = True
            print(f"Stream connected: {_fw}×{_fh}", flush=True)

        # ── Stream disconnect ──────────────────────────────────────────────────
        if stream_ok[0] and not r.stream_alive:
            stream_ok[0] = False
            cap = None
            _conn.update({'cap': None, 'ok': False, 'busy': False})
            print("Stream lost — showing standby display", flush=True)

        # ── Upload only changed panels to GPU ──────────────────────────────────
        panels = r.latest_panels()
        if panels is not None:
            vid, side, stats, vs, ss, sts = panels
            if vs  != _seen[0]:
                dpg.set_value("tex_vid",   _to_tex(vid))
                _seen[0] = vs
            if ss  != _seen[1]:
                dpg.set_value("tex_side",  _to_tex(side))
                _seen[1] = ss
            if sts != _seen[2]:
                dpg.set_value("tex_stats", _to_tex(stats))
                _seen[2] = sts

        dpg.render_dearpygui_frame()

    r = renderer[0]
    r.release()
    _macos_set_presentation(False)  # restore menu bar + Dock
    dpg.destroy_context()
    _target_sock.close()
    print(f"\nDone. {r.frame_count} frames processed.", flush=True)


if __name__ == '__main__':
    main()
