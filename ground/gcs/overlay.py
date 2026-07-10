"""
Pre-arm overlay: dims the GCS, shows a readiness checklist + settings, then arms.

Call open_overlay() to show it. tick(armed) each frame. draw_overlay() returns
a full-screen RGBA float32 array for upload to a DPG texture drawn on top of
the existing panels.
"""
import math
import time

import cv2
import numpy as np

try:
    from PIL import ImageFont, ImageDraw, Image as _PILImg
    _FONT_PATHS = [
        '/System/Library/Fonts/Helvetica.ttc',
        '/System/Library/Fonts/SFNSText.ttf',
        '/Library/Fonts/Arial.ttf',
    ]
    _FCACHE: dict[int, ImageFont.FreeTypeFont] = {}

    def _fnt(pt: int) -> ImageFont.FreeTypeFont:
        if pt not in _FCACHE:
            for p in _FONT_PATHS:
                try:
                    _FCACHE[pt] = ImageFont.truetype(p, pt)
                    _FCACHE[pt].getbbox('A')
                    break
                except Exception:
                    _FCACHE.pop(pt, None)
            else:
                _FCACHE[pt] = ImageFont.load_default()
        return _FCACHE[pt]

    _PIL = True
except ImportError:
    _PIL = False

import config
from mavlink import _mav_lock, _mav_state, send_mavlink_command, send_set_home

# Layout (logical px)
_MW     = 440
_HDR_H  = 44
_PAD    = 16
_ITEM_H = 30

_SEC1_Y = 68                                   # 24 px below header line
_ROW1_Y = _SEC1_Y + 30                         # 98 - 30 px from label to first row
_N_ROWS = 7                                    # LINK GPS IMU BARO BATTERY THROTTLE HOME
_SEC2_Y = _ROW1_Y + _N_ROWS * _ITEM_H + 42    # 350 - 42 px gap before OPTIONS
_OPT1_Y = _SEC2_Y + 32                        # 382 - 32 px from label to first option
_OPT2_Y = _OPT1_Y + _ITEM_H                   # 412
_ARM_Y  = _OPT2_Y + _ITEM_H + 20              # 462
_ARM_H  = 44
_CNL_Y  = _ARM_Y + _ARM_H + 12                # 518
_CNL_H  = 28
_MH     = _CNL_Y + _CNL_H + 16                # 562

_BG_DIM = 0.62
_FADE_S = 0.75

# State (main thread only)
_st: dict = {
    'active':    False,
    'arm_state': 'idle',   # 'idle' | 'arming' | 'fading'
    'fade_t':    0.0,
    'fade_a':    1.0,
    'rec':       False,
    'verb':      False,
}

# Click areas in logical screen coords - set during draw_overlay()
_HIT: dict = {}

# Per-frame text queue - batched into one PIL pass per draw call
_TQ: list = []


def _qt(text: str, xy: tuple, pt: int, bgr: tuple) -> None:
    _TQ.append((text, xy, pt, bgr))


def _flush(img: np.ndarray) -> None:
    if not _TQ:
        return
    if _PIL:
        pil  = _PILImg.fromarray(img[:, :, ::-1])
        draw = ImageDraw.Draw(pil)
        for text, xy, pt, bgr in _TQ:
            fnt = _fnt(pt)
            try:
                asc = fnt.getmetrics()[0]
            except AttributeError:
                asc = pt * 3 // 4
            draw.text((xy[0], xy[1] - asc), text, font=fnt,
                      fill=(bgr[2], bgr[1], bgr[0]))
        img[:, :, ::-1] = np.array(pil)
    else:
        for text, xy, pt, bgr in _TQ:
            cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX,
                        pt / 20.0, bgr, 1, cv2.LINE_AA)
    _TQ.clear()


def _sz(text: str, pt: int) -> tuple[int, int]:
    if _PIL:
        try:
            b = _fnt(pt).getbbox(text)
            return b[2] - b[0], b[3] - b[1]
        except Exception:
            pass
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, pt / 20.0, 1)
    return w, h


# Public API

def is_active() -> bool:
    return _st['active']


def open_overlay() -> None:
    _st.update(active=True, arm_state='idle', fade_a=1.0, fade_t=0.0)
    _st['rec']  = config.RECORD_ACTIVE
    _st['verb'] = config.VERBOSE_LOGGING


def _close() -> None:
    _st['active'] = False


def tick(armed: bool) -> None:
    """Call once per frame on the main thread."""
    if not _st['active']:
        return
    now = time.monotonic()
    if _st['arm_state'] == 'arming' and armed:
        _st['arm_state'] = 'fading'
        _st['fade_t']    = now
    if _st['arm_state'] == 'fading':
        _st['fade_a'] = max(0.0, 1.0 - (now - _st['fade_t']) / _FADE_S)
        if _st['fade_a'] <= 0.0:
            _close()


def handle_click(lx: int, ly: int) -> bool:
    """Route a logical-pixel screen click. Returns True if the overlay consumed it."""
    if not _st['active']:
        return False
    if _st['arm_state'] == 'fading':
        return True  # block passthrough during fade

    def _in(key: str) -> bool:
        r = _HIT.get(key)
        return bool(r and r[0] <= lx <= r[0] + r[2] and r[1] <= ly <= r[1] + r[3])

    mx, my, mw, mh = _HIT.get('modal', (0, 0, 0, 0))
    if not (mx <= lx <= mx + mw and my <= ly <= my + mh):
        _close()
        return True

    if _in('set_home'):
        send_set_home()
    elif _in('chk_rec'):
        _st['rec'] = not _st['rec']
        config.RECORD_ACTIVE = _st['rec']
    elif _in('chk_verb'):
        _st['verb'] = not _st['verb']
        config.VERBOSE_LOGGING = _st['verb']
    elif _in('arm') and _st['arm_state'] == 'idle':
        _st['arm_state'] = 'arming'
        send_mavlink_command(400, 1.0, 0.0)
    elif _in('cancel'):
        _close()
    return True


# Rendering

def draw_overlay(screen_w: int, screen_h: int) -> np.ndarray:
    """Return a (screen_h*PR, screen_w*PR, 4) float32 RGBA array for DPG upload."""
    _TQ.clear()
    s  = config.PR
    W  = screen_w * s
    H  = screen_h * s
    fa = _st['fade_a']

    canvas = np.zeros((H, W, 4), dtype=np.float32)
    if not _st['active']:
        return canvas.ravel()

    # Background scrim
    canvas[:, :, 3] = _BG_DIM * fa

    # Telemetry snapshot
    now = time.monotonic()
    with _mav_lock:
        last_t       = _mav_state['last_msg_t']
        gps_fix      = _mav_state['gps_fix_type']
        sensors      = _mav_state['sensors_health']
        batt_pct     = _mav_state['battery_pct']
        throttle_pct = _mav_state['throttle_pct']
        origin       = _mav_state['origin']
        home_set     = _mav_state['home_set']
    linked = last_t is not None and (now - last_t) < 2.0

    # Modal geometry
    mw = _MW * s
    mh = _MH * s
    mx = (W - mw) // 2
    my = (H - mh) // 2

    # Logical hit areas (all relative to screen top-left)
    lmx, lmy = mx // s, my // s
    _HIT['modal']    = (lmx,                lmy,              _MW,          _MH)
    _HIT['chk_rec']  = (lmx + _PAD,         lmy + _OPT1_Y,    _MW - 2*_PAD, _ITEM_H)
    _HIT['chk_verb'] = (lmx + _PAD,         lmy + _OPT2_Y,    _MW - 2*_PAD, _ITEM_H)
    _HIT['arm']      = (lmx + _PAD,         lmy + _ARM_Y,     _MW - 2*_PAD, _ARM_H)
    _HIT['cancel']   = (lmx + (_MW-140)//2, lmy + _CNL_Y,     140,          _CNL_H)

    # Modal surface (BGR uint8)
    modal = np.full((mh, mw, 3), (44, 46, 56), dtype=np.uint8)

    # Header bar
    cv2.rectangle(modal, (0, 0), (mw, _HDR_H * s), (52, 56, 70), -1)
    cv2.line(modal, (0, _HDR_H * s), (mw, _HDR_H * s), (65, 70, 88), max(1, s))

    title_pt = max(10, 14 * s)
    tw, _ = _sz('PRE-FLIGHT CHECKLIST', title_pt)
    _qt('PRE-FLIGHT CHECKLIST', ((mw - tw) // 2, 28 * s), title_pt, (205, 212, 225))

    # Readiness section
    sec_pt  = max(10, 10 * s)
    body_pt = max(10, 12 * s)

    _qt('READINESS', (_PAD * s, _SEC1_Y * s), sec_pt, (95, 100, 118))
    cv2.line(modal, (_PAD * s, (_SEC1_Y + 6) * s), (mw - _PAD * s, (_SEC1_Y + 6) * s),
             (58, 62, 76), max(1, s))

    imu_ok      = (sensors & 0x03 == 0x03) if sensors else False
    baro_ok     = bool(sensors & 0x08)      if sensors else False
    thr_ok      = linked and throttle_pct < 5
    home_ok     = home_set

    checklist = [
        ('LINK',
         'LIVE'      if linked       else 'NO LINK',
         linked),
        ('GPS',
         '3D FIX'    if gps_fix >= 3 else
         'ACQUIRING' if gps_fix >= 1 else 'NO GPS',
         gps_fix >= 3),
        ('IMU',
         'OK'        if imu_ok      else
         '--'        if not sensors else 'FAULT',
         imu_ok),
        ('BARO',
         'OK'        if baro_ok     else
         '--'        if not sensors else 'FAULT',
         baro_ok),
        ('BATTERY',
         f'{batt_pct}%  OK'  if batt_pct > 30 else
         f'{batt_pct}%  LOW' if batt_pct >= 0 else '--',
         batt_pct > 30),
        ('THROTTLE',
         'ZERO'           if thr_ok                    else
         f'{throttle_pct}%  HIGH' if linked            else '--',
         thr_ok),
        ('HOME',
         'LOCKED'    if home_ok else 'NOT SET',
         home_ok),
    ]
    all_ready = all(ok for _, _, ok in checklist)

    for i, (lbl, status, ok) in enumerate(checklist):
        ry     = (_ROW1_Y + i * _ITEM_H) * s
        mid_y  = ry + _ITEM_H * s // 2
        dot    = (55, 210, 80) if ok else (62, 62, 215)
        st_col = (80, 215, 100) if ok else (85, 85, 220)
        cv2.circle(modal, (_PAD * s + 5 * s, mid_y), 4 * s, dot, -1)
        _qt(lbl, ((_PAD + 16) * s, mid_y + 6 * s), body_pt, (162, 168, 180))

        if lbl == 'HOME' and not home_ok:
            # "SET HOME" button replaces the status text
            btn_pt = max(8, 9 * s)
            btn_w  = 76 * s
            btn_h  = 20 * s
            bx0    = mw - btn_w - _PAD * s
            by0    = ry + (_ITEM_H * s - btn_h) // 2
            bx1    = bx0 + btn_w
            by1    = by0 + btn_h
            has_gps = gps_fix >= 3
            btn_bg  = (50, 70, 38)   if has_gps else (42, 42, 50)
            btn_bd  = (80, 140, 60)  if has_gps else (58, 60, 70)
            btn_fg  = (110, 210, 85) if has_gps else (80, 82, 95)
            cv2.rectangle(modal, (bx0, by0), (bx1, by1), btn_bg, -1)
            cv2.rectangle(modal, (bx0, by0), (bx1, by1), btn_bd, max(1, s))
            bw, _ = _sz('SET HOME', btn_pt)
            _qt('SET HOME', (bx0 + (btn_w - bw) // 2, by0 + (btn_h + btn_pt) // 2),
                btn_pt, btn_fg)
            _HIT['set_home'] = (lmx + bx0 // s, lmy + ry // s, btn_w // s, _ITEM_H)
        else:
            sw, _ = _sz(status, body_pt)
            _qt(status, (mw - sw - _PAD * s, mid_y + 6 * s), body_pt, st_col)
            _HIT.pop('set_home', None)

        if i < _N_ROWS - 1:
            cv2.line(modal, (_PAD * s, ry + _ITEM_H * s),
                     (mw - _PAD * s, ry + _ITEM_H * s), (50, 52, 64), max(1, s))

    # Options section
    cv2.line(modal, (_PAD * s, (_SEC2_Y - 28) * s), (mw - _PAD * s, (_SEC2_Y - 28) * s),
             (65, 70, 88), max(1, s))
    _qt('OPTIONS', (_PAD * s, _SEC2_Y * s), sec_pt, (95, 100, 118))
    cv2.line(modal, (_PAD * s, (_SEC2_Y + 6) * s), (mw - _PAD * s, (_SEC2_Y + 6) * s),
             (58, 62, 76), max(1, s))

    for opt_y, key, label in [(_OPT1_Y, 'rec', 'Save Recording'),
                               (_OPT2_Y, 'verb', 'Verbose Logging')]:
        mid_y   = (opt_y + _ITEM_H // 2) * s
        checked = _st[key]
        bx0 = _PAD * s
        by0 = (opt_y + (_ITEM_H - 14) // 2) * s
        bsz = 14 * s
        bx1 = bx0 + bsz
        by1 = by0 + bsz
        cv2.rectangle(modal, (bx0, by0), (bx1, by1), (60, 64, 80), max(1, s))
        if checked:
            cv2.rectangle(modal, (bx0 + 2*s, by0 + 2*s), (bx1 - 2*s, by1 - 2*s),
                          (75, 195, 105), -1)
            p1 = (bx0 + 3 * s,         by0 + bsz // 2)
            p2 = (bx0 + bsz // 2 - s,  by1 - 3 * s)
            p3 = (bx1 - 2 * s,          by0 + 3 * s)
            cv2.line(modal, p1, p2, (230, 240, 230), max(1, 2 * s))
            cv2.line(modal, p2, p3, (230, 240, 230), max(1, 2 * s))
        _qt(label, ((_PAD + 20) * s, mid_y + 6 * s), body_pt, (168, 172, 185))

    # ARM button
    arm_state = _st['arm_state']
    ax0 = _PAD * s
    ay0 = _ARM_Y * s
    ax1 = mw - _PAD * s
    ay1 = (_ARM_Y + _ARM_H) * s

    if arm_state == 'arming':
        pulse  = 0.55 + 0.45 * abs(math.sin(now * math.pi * 1.5))
        arm_bg = (int(20 * pulse), int(100 * pulse), int(130 * pulse))
        arm_bd = (int(40 * pulse), int(170 * pulse), int(210 * pulse))
        arm_fg = (int(80 * pulse), int(220 * pulse), int(255 * pulse))
        arm_lbl = 'ARMING…'
    elif all_ready:
        arm_bg  = (32, 95, 38)
        arm_bd  = (55, 175, 70)
        arm_fg  = (95, 235, 110)
        arm_lbl = 'ARM'
    else:
        arm_bg  = (38, 60, 40)
        arm_bd  = (55, 88, 58)
        arm_fg  = (72, 122, 76)
        arm_lbl = 'ARM'

    cv2.rectangle(modal, (ax0, ay0), (ax1, ay1), arm_bg, -1)
    cv2.rectangle(modal, (ax0, ay0), (ax1, ay1), arm_bd, max(1, s))
    arm_pt = max(10, 17 * s)
    aw, _  = _sz(arm_lbl, arm_pt)
    _qt(arm_lbl, ((mw - aw) // 2, ay0 + (_ARM_H * s + arm_pt) // 2 - s),
        arm_pt, arm_fg)

    # Cancel
    cnl_pt = max(10, 11 * s)
    cw, _  = _sz('Cancel', cnl_pt)
    _qt('Cancel', ((mw - cw) // 2, (_CNL_Y + _CNL_H // 2 + 6) * s),
        cnl_pt, (95, 98, 116))

    # Flush text and composite into RGBA canvas
    _flush(modal)

    canvas[my:my + mh, mx:mx + mw, 0] = modal[:, :, 2] / 255.0
    canvas[my:my + mh, mx:mx + mw, 1] = modal[:, :, 1] / 255.0
    canvas[my:my + mh, mx:mx + mw, 2] = modal[:, :, 0] / 255.0
    canvas[my:my + mh, mx:mx + mw, 3] = fa

    return canvas.ravel()
