import math
import time
import os

import cv2
import numpy as np

import config
import gfx
from config import PROGRAMS, PAYLOAD_NAMES, OVERLAY_H
from mavlink import _mav_lock, _mav_state, send_mavlink_command

_FLIGHT_MODES = {
    0: 'STAB', 1: 'ALTH', 2: 'POSH', 3: 'AUTO', 4: 'RTH', 5: 'LAND', 6: 'FOLW',
}
_FLIGHT_STATES = {
    0: 'IDLE', 1: 'ARMING', 2: 'ARMED', 3: 'FLYING', 4: 'LANDING', 5: 'FAULT',
}
_STATE_COLORS = {
    0: (90,  90,  90),
    1: (30, 160, 200),
    2: (30, 160, 200),
    3: (80, 200,  80),
    4: (40, 140, 220),
    5: (40,  40, 210),
}

# Maps PROGRAMS index → (cmd, param1, param2) sent on SEND. DO_SET_MODE param2
# = firmware FlightMode discriminant, same numbering as HEARTBEAT custom_mode
# (0=Stab 1=AltH 2=PosH 3=Auto 4=RTH 5=Land 6=FollowMe).
_PROG_COMMANDS = {
    0: (176, 0.0, 0.0),   # MANUAL      → DO_SET_MODE, Stabilise
    1: (176, 0.0, 6.0),   # FOLLOW ME   → DO_SET_MODE, FollowMe
    2: (176, 0.0, 3.0),   # WEED PICKER → DO_SET_MODE, Auto
    3: (20,  0.0, 0.0),   # RETURN HOME → NAV_RETURN_TO_LAUNCH
}

_ui_state: dict = {
    'selected_prog': 0, 'running_prog': None, 'stopped': False, 'landing': False,
    'pending_ack_prog': None,
    'denied_prog':      None,
    'denied_until':     0.0,
}

# ── Bench motor test (MAV_CMD_DO_MOTOR_TEST, 209) ────────────────────────────
# A row of M1..M4 buttons under the SEND/STOP buttons. The firmware spins one
# motor for 2 s, ONLY while disarmed and on the ground (Idle), and clamps the
# throttle to 0.20. PROPS OFF — this exists to verify per-corner wiring + spin
# direction during bring-up.
MOTOR_TEST_THROTTLE = 0.08   # 8 % — clear spin, well under the firmware's 0.20 clamp
# Logical-pixel geometry of the program panel's button rows, relative to the panel
# top. Shared by _draw_program_panel (× config.PR when drawing) and
# _handle_overlay_click (× 1 when hit-testing) so buttons and hit-boxes stay aligned.
_PROG_SEP_Y  = 28 + len(PROGRAMS) * 26 + 4   # separator under the program rows (136)
_PROG_BTN_Y0 = _PROG_SEP_Y + 6               # SEND/STOP/LAND/RESUME button top  (142)
_PROG_BTN_Y1 = _PROG_SEP_Y + 30              # ...and bottom                      (166)
_MT_BTN_Y0   = _PROG_BTN_Y1 + 18             # motor-test M1..M4 button top       (184)
_MT_BTN_Y1   = _MT_BTN_Y0 + 26               # ...and bottom                      (210)


def _motor_test_btn_xranges(w: int, unit: float):
    """X-ranges (x0, x1) of the four M1..M4 buttons, from the panel's left edge.
    Pass unit=config.PR with the physical width when drawing, or unit=1 with the
    logical width when hit-testing — the shared ratio keeps the two aligned."""
    margin = 8 * unit
    gap    = 4 * unit
    cell   = (w - 2 * margin - 3 * gap) / 4
    return [(int(round(margin + i * (cell + gap))),
             int(round(margin + i * (cell + gap) + cell))) for i in range(4)]

_payload_disabled: set = set()
_payload_panel_rect: tuple = (0, 0, 0, 0)   # logical coords for click detection
_arm_btn_rect: tuple = (0, 0, 0, 0)         # logical stats-panel coords for ARM/DISARM btn


def get_arm_btn_rect() -> tuple:
    return _arm_btn_rect


def toggle_payload_override(bit: int) -> None:
    if bit in _payload_disabled:
        _payload_disabled.discard(bit)
        print(f"Payload '{PAYLOAD_NAMES.get(bit)}' re-enabled", flush=True)
    else:
        _payload_disabled.add(bit)
        print(f"Payload '{PAYLOAD_NAMES.get(bit)}' disabled", flush=True)


def handle_payload_double_click(px: int, py: int) -> None:
    """Toggle a payload override when the operator double-clicks its row."""
    rx, ry, rw, rh = _payload_panel_rect
    if not (rx <= px <= rx + rw and ry <= py <= ry + rh):
        return
    rel_y = py - ry - 25
    if rel_y < 0:
        return
    idx = rel_y // 24
    bits = sorted(PAYLOAD_NAMES.keys())
    if idx < len(bits):
        toggle_payload_override(bits[idx])


_horizon_img_cache: dict = {}   # r → {'sky': patch, 'gnd': patch}


def _load_horizon_patch(path: str, r: int) -> np.ndarray | None:
    """Load and centre-crop an RGBA image to a (2r × 2r) patch."""
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None or img.ndim < 3 or img.shape[2] != 4:
        return None
    target = 2 * r
    scale  = max(target / img.shape[1], target / img.shape[0])
    img    = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)),
                        interpolation=cv2.INTER_AREA)
    yo = (img.shape[0] - target) // 2
    xo = (img.shape[1] - target) // 2
    return img[yo:yo + target, xo:xo + target]


def _draw_artificial_horizon(panel: np.ndarray, cx: int, cy: int, r: int,
                              roll_rad: float, pitch_rad: float) -> None:
    s = config.PR
    sky_bgr = (150, 90, 50)
    gnd_bgr = (35, 100, 160)

    pixels_per_rad = r / math.radians(35)
    pitch_px = max(-r, min(r, int(pitch_rad * pixels_per_rad)))

    sky_x =  math.sin(roll_rad)
    sky_y = -math.cos(roll_rad)
    hdx   =  math.cos(roll_rad)
    hdy   =  math.sin(roll_rad)

    hx  = cx - int(pitch_px * sky_x)
    hy  = cy - int(pitch_px * sky_y)
    ext = r + 6 * s

    l  = (int(hx - ext * hdx), int(hy - ext * hdy))
    ri = (int(hx + ext * hdx), int(hy + ext * hdy))
    tl = (int(l[0]  + ext * sky_x), int(l[1]  + ext * sky_y))
    tr = (int(ri[0] + ext * sky_x), int(ri[1] + ext * sky_y))

    cv2.circle(panel, (cx, cy), r, gnd_bgr, -1)
    
    mask = np.zeros(panel.shape[:2], dtype=np.uint8)
    cv2.circle(mask, (cx, cy), r, 255, -1)
    
    tmp = panel.copy()
    
    sky_poly_mask = np.zeros(tmp.shape[:2], dtype=np.uint8)
    poly_points = np.array([l, ri, tr, tl], np.int32)
    cv2.fillPoly(sky_poly_mask, [poly_points], 255)
    
    # Fill the sky color onto 'tmp' using the polygon mask
    tmp[sky_poly_mask == 255] = sky_bgr

    # Populate cache for this radius on first use (or if r changed).
    if r not in _horizon_img_cache:
        _assets = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'assets')
        _horizon_img_cache[r] = {
            'sky': _load_horizon_patch(os.path.join(_assets, 'trees.png'), r),
            'gnd': _load_horizon_patch(os.path.join(_assets, 'path.png'),  r),
        }

    _sky_patch = _horizon_img_cache[r]['sky']
    _gnd_patch = _horizon_img_cache[r]['gnd']

    if _sky_patch is not None or _gnd_patch is not None:
        y1, y2 = cy - r, cy + r
        x1, x2 = cx - r, cx + r
        sky_poly_roi = sky_poly_mask[y1:y2, x1:x2]

        if _sky_patch is not None:
            sky_alpha = _sky_patch[:, :, 3].astype(float) / 255.0
            sky_alpha[sky_poly_roi == 0] = 0.0
            sky_alpha = np.expand_dims(sky_alpha, axis=2)
            tmp_roi = tmp[y1:y2, x1:x2]
            tmp[y1:y2, x1:x2] = (_sky_patch[:, :, :3] * sky_alpha
                                  + tmp_roi * (1.0 - sky_alpha)).astype(np.uint8)

        if _gnd_patch is not None:
            gnd_alpha = _gnd_patch[:, :, 3].astype(float) / 255.0
            gnd_alpha[sky_poly_roi != 0] = 0.0
            gnd_alpha = np.expand_dims(gnd_alpha, axis=2)
            tmp_roi = tmp[y1:y2, x1:x2]
            tmp[y1:y2, x1:x2] = (_gnd_patch[:, :, :3] * gnd_alpha
                                  + tmp_roi * (1.0 - gnd_alpha)).astype(np.uint8)
        
    panel[mask == 255] = tmp[mask == 255]

    for deg in [-20, -10, 10, 20]:
        off = math.radians(deg) * pixels_per_rad
        px  = int(hx - off * sky_x)
        py  = int(hy - off * sky_y)
        hw  = r // 4 if abs(deg) == 20 else r // 6
        cv2.line(panel,
                 (int(px - hw * hdx), int(py - hw * hdy)),
                 (int(px + hw * hdx), int(py + hw * hdy)),
                 (200, 200, 200), max(1, s))

    cv2.line(panel, l, ri, (240, 240, 240), max(1, 2 * s))

    roll_deg = math.degrees(roll_rad)
    cv2.ellipse(panel, (cx, cy), (r - 5 * s, r - 5 * s), 0, -140, -40, (160, 160, 160), max(1, s))
    ta  = math.radians(-90 - roll_deg)
    tip = (int(cx + (r - 5 * s) * math.cos(ta)), int(cy + (r - 5 * s) * math.sin(ta)))
    tl2 = (int(cx + (r + 3 * s) * math.cos(ta - 0.15)), int(cy + (r + 3 * s) * math.sin(ta - 0.15)))
    tr2 = (int(cx + (r + 3 * s) * math.cos(ta + 0.15)), int(cy + (r + 3 * s) * math.sin(ta + 0.15)))
    cv2.fillPoly(panel, [np.array([tip, tl2, tr2], np.int32)], (0, 220, 255))

    hw = r // 3
    cv2.line(panel, (cx - hw, cy), (cx - hw // 3, cy), (0, 220, 255), max(1, 2 * s))
    cv2.line(panel, (cx + hw // 3, cy), (cx + hw, cy), (0, 220, 255), max(1, 2 * s))
    cv2.circle(panel, (cx, cy), 3 * s, (0, 220, 255), -1)
    cv2.circle(panel, (cx, cy), r, (90, 90, 90), max(1, 2 * s))


def _draw_compass(panel: np.ndarray, cx: int, cy: int, r: int,
                  heading_deg: float) -> None:
    s = config.PR
    cv2.circle(panel, (cx, cy), r, (45, 45, 45), -1)
    cv2.circle(panel, (cx, cy), r, (90, 90, 90), max(1, s))

    for deg in range(0, 360, 45):
        a     = math.radians(deg - heading_deg - 90)
        inner = r - (9 * s if deg % 90 == 0 else 5 * s)
        p1 = (int(cx + inner * math.cos(a)), int(cy + inner * math.sin(a)))
        p2 = (int(cx + (r - s) * math.cos(a)), int(cy + (r - s) * math.sin(a)))
        cv2.line(panel, p1, p2, (130, 130, 130), max(1, s))

    for deg, lbl in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        a  = math.radians(deg - heading_deg - 90)
        lx = int(cx + (r - 16 * s) * math.cos(a)) - 4 * s
        ly = int(cy + (r - 16 * s) * math.sin(a)) + 4 * s
        col = (60, 60, 220) if lbl == 'N' else (190, 190, 190)
        gfx.put_text(panel, lbl, (lx, ly), 0.36 * s, col)

    tip = (cx, cy - r + 3 * s)
    cv2.fillPoly(panel,
                 [np.array([tip,
                             (cx - 5 * s, cy - r + 13 * s),
                             (cx + 5 * s, cy - r + 13 * s)], np.int32)],
                 (60, 180, 255))
    hdg_s = f"{int(heading_deg) % 360:03d}"
    (tw, _), _ = gfx.size(hdg_s, 0.42 * s)
    gfx.put_text(panel, hdg_s, (cx - tw // 2, cy + 8 * s), 0.42 * s, (220, 220, 220))


def _draw_arc_gauge(panel: np.ndarray, cx: int, cy: int, r: int,
                    value: float, vmin: float, vmax: float,
                    label: str, unit: str, arc_color: tuple,
                    warn_frac: float | None = None) -> None:
    """270° sweep gauge: lower-left (135°) → top → lower-right (405°=45°)."""
    s = config.PR
    cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 405, (55, 55, 55), max(1, 6 * s))

    frac  = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
    color = (40, 100, 230) if (warn_frac and frac >= warn_frac) else arc_color
    if frac > 0:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 135 + frac * 270, color, max(1, 5 * s))

    na   = math.radians(135 + frac * 270)
    ntip = (int(cx + (r - 7 * s) * math.cos(na)), int(cy + (r - 7 * s) * math.sin(na)))
    cv2.line(panel, (cx, cy), ntip, (220, 220, 220), max(1, 2 * s))
    cv2.circle(panel, (cx, cy), 4 * s, (170, 170, 170), -1)

    val_s = (f"{value:.0f}" if abs(value) >= 10 else f"{value:.1f}") + unit
    (tw, _), _ = gfx.size(val_s, 0.40 * s)
    gfx.put_text(panel, val_s, (cx - tw // 2, cy + r + 1 * s), 0.40 * s, (215, 215, 215))
    (lw, _), _ = gfx.size(label, 0.32 * s)
    gfx.put_text(panel, label, (cx - lw // 2, cy + r + 14 * s), 0.32 * s, (105, 105, 105))


def _draw_vspeed(panel: np.ndarray, cx: int, cy: int, r: int,
                 climb_ms: float) -> None:
    """Vertical speed indicator: ±5 m/s, zero at top of arc (270°)."""
    s = config.PR
    MAX_MS = 5.0
    cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 405, (55, 55, 55), max(1, 6 * s))
    cv2.line(panel, (cx, cy - r + 3 * s), (cx, cy - r + 11 * s),
             (160, 160, 160), max(1, 2 * s))

    frac   = max(-1.0, min(1.0, climb_ms / MAX_MS))
    na_deg = 270 + frac * 135
    na     = math.radians(na_deg)
    arc_col = (60, 200, 60) if frac >= 0 else (60, 60, 210)
    if frac >= 0:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, 270, na_deg, arc_col, max(1, 5 * s))
    else:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, na_deg, 270, arc_col, max(1, 5 * s))

    ntip = (int(cx + (r - 7 * s) * math.cos(na)), int(cy + (r - 7 * s) * math.sin(na)))
    cv2.line(panel, (cx, cy), ntip, (220, 220, 220), max(1, 2 * s))
    cv2.circle(panel, (cx, cy), 4 * s, (170, 170, 170), -1)

    sign  = '+' if climb_ms >= 0 else ''
    val_s = f"{sign}{climb_ms:.1f}m/s"
    (tw, _), _ = gfx.size(val_s, 0.38 * s)
    gfx.put_text(panel, val_s, (cx - tw // 2, cy + r + 1 * s), 0.38 * s, (200, 200, 200))
    (lw, _), _ = gfx.size('V.SPEED', 0.32 * s)
    gfx.put_text(panel, 'V.SPEED', (cx - lw // 2, cy + r + 14 * s), 0.32 * s, (105, 105, 105))


def _draw_battery(panel: np.ndarray, x: int, y: int, w: int, h: int,
                  pct: int) -> None:
    s = config.PR
    cv2.rectangle(panel, (x, y), (x + w, y + h), (70, 70, 70), max(1, s))
    if pct >= 0:
        fill_w = max(1, int(w * pct / 100))
        green  = min(255, int(510 * pct / 100))
        red    = min(255, int(510 * (100 - pct) / 100))
        cv2.rectangle(panel, (x + s, y + s), (x + fill_w, y + h - s), (0, green, red), -1)
    pct_s = f"{pct}%" if pct >= 0 else "--"
    gfx.put_text(panel, pct_s, (x + w + 6 * s, y + h - s), 0.36 * s, (170, 170, 170))


def _draw_battery_panel(panel: np.ndarray, x: int, y: int, w: int,
                        voltage_mv: int, current_ca: int, battery_pct: int,
                        mah_consumed: int, time_remaining_s: int) -> None:
    s = config.PR
    P = 10 * s
    gfx.put_text(panel, 'POWER', (x + P, y + 16 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 22 * s), (x + w, y + 22 * s), (48, 48, 48), max(1, s))

    cy = y + 38 * s
    v_s = f"{voltage_mv / 1000:.1f}V" if voltage_mv > 0  else "--V"
    a_s = f"{current_ca / 100:.1f}A"  if current_ca >= 0 else "--A"
    gfx.put_text(panel, v_s, (x + P, cy + 20 * s), 0.80 * s, (200, 210, 100))
    (aw, _), _ = gfx.size(a_s, 0.80 * s)
    gfx.put_text(panel, a_s, (x + w - aw - P, cy + 20 * s), 0.80 * s, (100, 180, 220))
    gfx.put_text(panel, 'VOLT', (x + P, cy + 34 * s), 0.30 * s, (70, 70, 70))
    gfx.put_text(panel, 'CURR', (x + w - 30 * s - P, cy + 34 * s), 0.30 * s, (70, 70, 70))
    cy += 44 * s

    gfx.put_text(panel, 'CHARGE', (x + P, cy + 11 * s), 0.34 * s, (95, 95, 95))
    _draw_battery(panel, x + 65 * s, cy, w - 95 * s - P, 13 * s, battery_pct)
    cy += 24 * s

    t_s = (f"{divmod(time_remaining_s, 60)[0]}:{divmod(time_remaining_s, 60)[1]:02d}"
           if time_remaining_s > 0 else "--:--")
    gfx.put_text(panel, 'BATT TIME', (x + P, cy + 11 * s), 0.34 * s, (95, 95, 95))
    (tw, _), _ = gfx.size(t_s, 0.50 * s)
    gfx.put_text(panel, t_s, (x + w - tw - P, cy + 13 * s), 0.50 * s, (200, 200, 200))
    cy += 24 * s

    mah_s = f"{mah_consumed:.0f} mAh" if mah_consumed >= 0 else "-- mAh"
    gfx.put_text(panel, 'CONSUMED', (x + P, cy + 11 * s), 0.34 * s, (95, 95, 95))
    (mw, _), _ = gfx.size(mah_s, 0.42 * s)
    gfx.put_text(panel, mah_s, (x + w - mw - P, cy + 12 * s), 0.42 * s, (170, 170, 170))


def _draw_distance_home(panel: np.ndarray, x: int, y: int, w: int,
                        lat, lon, origin) -> None:
    s = config.PR
    P = 10 * s
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), max(1, s))
    gfx.put_text(panel, 'NAVIGATION', (x + P, y + 18 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 24 * s), (x + w, y + 24 * s), (48, 48, 48), max(1, s))

    if origin is not None and lat is not None:
        R     = 6_371_000.0
        lat0, lon0 = origin
        east  = math.radians(lon  - lon0) * R * math.cos(math.radians(lat0))
        north = math.radians(lat  - lat0) * R
        dist  = math.hypot(east, north)
        bear  = (math.degrees(math.atan2(east, north)) + 360) % 360
        dist_s = f"{dist:.1f} m"
        bear_s = f"{bear:.0f}"
    else:
        dist_s = "-- m"
        bear_s = "--"

    cy = y + 44 * s
    gfx.put_text(panel, 'DIST HOME', (x + P, cy), 0.34 * s, (95, 95, 95))
    (dw, _), _ = gfx.size(dist_s, 0.68 * s)
    gfx.put_text(panel, dist_s, (x + w - dw - P, cy + 2 * s), 0.68 * s, (215, 195, 130))
    cy += 30 * s
    gfx.put_text(panel, 'BEARING', (x + P, cy), 0.34 * s, (95, 95, 95))
    (bw, _), _ = gfx.size(bear_s, 0.55 * s)
    gfx.put_text(panel, bear_s, (x + w - bw - P, cy + 2 * s), 0.55 * s, (185, 185, 185))
    cy += 26 * s
    gfx.put_text(panel,
                 f"LAT  {lat:.5f}" if lat is not None else "LAT  --",
                 (x + P, cy), 0.34 * s, (128, 128, 128))
    cy += 18 * s
    gfx.put_text(panel,
                 f"LON  {lon:.5f}" if lon is not None else "LON  --",
                 (x + P, cy), 0.34 * s, (128, 128, 128))
    cy += 20 * s
    cruise_s = f"{config.CRUISE_ALT:.1f} m"
    gfx.put_text(panel, 'CRUISE ALT', (x + P, cy), 0.34 * s, (95, 95, 95))
    (caw, _), _ = gfx.size(cruise_s, 0.44 * s)
    gfx.put_text(panel, cruise_s, (x + w - caw - P, cy), 0.44 * s, (140, 175, 210))


_DOT_COLORS = {
    'OK':    ( 50, 200,  70),
    'CHECK': ( 50, 185, 255),
    'FAULT': ( 55,  55, 210),
    '--':    ( 50,  50,  50),
}


def _draw_status_pane(panel: np.ndarray, x: int, y: int, w: int, h: int,
                      statuses: list) -> None:
    s = config.PR
    P = 6 * s
    gfx.put_text(panel, 'STATUS', (x + P, y + 16 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 22 * s), (x + w, y + 22 * s), (48, 48, 48), max(1, s))

    body_y = y + 23 * s
    n      = len(statuses)
    row_h  = (h - 21 * s) // n

    for i, (label, status) in enumerate(statuses):
        ry = body_y + i * row_h + row_h // 2
        if label is None:
            continue
        col = _DOT_COLORS.get(status, _DOT_COLORS['--'])
        cv2.circle(panel, (x + P + 3 * s, ry), 3 * s, col, -1)
        gfx.put_text(panel, label, (x + P + 10 * s, ry + 4 * s), 0.30 * s, (100, 105, 110))
        (sw, _), _ = gfx.size(status, 0.30 * s)
        gfx.put_text(panel, status, (x + w - sw - P, ry + 4 * s), 0.30 * s, col)
        next_label = statuses[i + 1][0] if i < n - 1 else None
        if i < n - 1 and next_label is not None:
            cv2.line(panel, (x + 4 * s, body_y + (i + 1) * row_h),
                     (x + w - 4 * s, body_y + (i + 1) * row_h), (38, 38, 38), max(1, s))


def _draw_motors(panel: np.ndarray, x: int, y: int, w: int, h: int,
                 motor_pwm: list, armed: bool, throttle_pct: int = 0) -> None:
    s = config.PR
    P = 10 * s
    gfx.put_text(panel, 'MOTORS', (x + P, y + 16 * s), 0.40 * s, (95, 95, 95))
    thr_s = f"THR {throttle_pct}%"
    (tw, _), _ = gfx.size(thr_s, 0.32 * s)
    gfx.put_text(panel, thr_s, (x + w - tw - P, y + 16 * s), 0.32 * s, (80, 80, 80))
    cv2.line(panel, (x, y + 22 * s), (x + w, y + 22 * s), (48, 48, 48), max(1, s))

    dh  = h - 22 * s
    dcx = x + w // 2
    dcy = y + 22 * s + dh // 2
    arm = min(w // 2, dh // 2) - 20 * s

    positions = [
        (dcx - arm, dcy - arm),
        (dcx + arm, dcy - arm),
        (dcx - arm, dcy + arm),
        (dcx + arm, dcy + arm),
    ]
    for mx, my in positions:
        cv2.line(panel, (dcx, dcy), (mx, my), (58, 58, 58), max(1, 2 * s))

    cv2.circle(panel, (dcx, dcy), 8 * s, (88, 88, 88), -1)
    cv2.circle(panel, (dcx, dcy), 8 * s, (115, 115, 115), max(1, s))
    cv2.fillPoly(panel,
                 [np.array([(dcx - 4 * s, dcy - 8 * s),
                             (dcx + 4 * s, dcy - 8 * s),
                             (dcx,         dcy - 15 * s)], np.int32)],
                 (90, 170, 255))

    mr = 14 * s
    for i, (mx, my) in enumerate(positions):
        pwm = motor_pwm[i] if i < len(motor_pwm) else 0
        if not armed or pwm <= 1050:
            ring = (55, 55, 55)
        elif pwm <= 1350:
            ring = (40, 185, 40)
        elif pwm <= 1650:
            ring = (0, 210, 220)
        else:
            ring = (40, 40, 210)
        cv2.circle(panel, (mx, my), mr, (33, 33, 33), -1)
        cv2.circle(panel, (mx, my), mr, ring, max(1, 2 * s))
        lbl = f"M{i + 1}"
        (lw, lh), _ = gfx.size(lbl, 0.30 * s)
        gfx.put_text(panel, lbl, (mx - lw // 2, my + lh // 2), 0.30 * s, (158, 158, 158))
        pwm_s = str(pwm) if pwm > 0 else "--"
        (pw, _), _ = gfx.size(pwm_s, 0.27 * s)
        lbl_y = my + mr + 11 * s if my >= dcy else my - mr - 3 * s
        gfx.put_text(panel, pwm_s, (mx - pw // 2, lbl_y), 0.27 * s, (88, 88, 88))


def _draw_payloads(panel: np.ndarray, x: int, y: int, w: int,
                   payload_flags: int, linked: bool) -> None:
    s = config.PR
    P = 10 * s
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), max(1, s))
    gfx.put_text(panel, 'PAYLOADS', (x + P, y + 18 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 24 * s), (x + w, y + 24 * s), (48, 48, 48), max(1, s))
    ry = y + 44 * s
    for bit, name in PAYLOAD_NAMES.items():
        manually_off = bit in _payload_disabled
        present      = linked and bool(payload_flags & (1 << bit)) and not manually_off
        if manually_off:
            dot_col, status_col, status_lbl = (30, 110, 160), (50, 160, 210), 'OFF'
        elif present:
            dot_col, status_col, status_lbl = (50, 200, 100), (130, 210, 130), 'OK'
        else:
            dot_col, status_col, status_lbl = (50, 40, 80),   (90, 65, 100),  '--'
        cv2.circle(panel, (x + P + 4 * s, ry - 4 * s), 4 * s, dot_col, -1)
        gfx.put_text(panel, name, (x + P + 14 * s, ry), 0.38 * s, (162, 162, 162))
        (sw, _), _ = gfx.size(status_lbl, 0.36 * s)
        gfx.put_text(panel, status_lbl, (x + w - sw - P, ry), 0.36 * s, status_col)
        ry += 24 * s


_MAP_MAX_SCALE   = 40.0
_MAP_MIN_SCALE   =  2.0
_MAP_DEFAULT_SCALE = 12.0


def _draw_weed_map(panel: np.ndarray, x: int, y: int, w: int, h: int,
                   lat, lon, yaw_rad: float, origin,
                   weed_world_pos: list, linked: bool) -> None:
    s = config.PR
    P = 10 * s
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), max(1, s))
    gfx.put_text(panel, 'MAP', (x + P, y + 18 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 24 * s), (x + w, y + 24 * s), (48, 48, 48), max(1, s))

    by = y + 25 * s
    bh = h - 25 * s
    cv2.rectangle(panel, (x, by), (x + w, by + bh), (16, 20, 16), -1)

    if not linked or origin is None or lat is None:
        (mw, _), _ = gfx.size('NO GPS', 0.40 * s)
        gfx.put_text(panel, 'NO GPS', (x + w // 2 - mw // 2, by + bh // 2),
                     0.40 * s, (55, 55, 55))
        return

    R_earth = 6_371_000.0
    lat0, lon0 = origin
    drone_e = math.radians(lon  - lon0) * R_earth * math.cos(math.radians(lat0))
    drone_n = math.radians(lat  - lat0) * R_earth

    if weed_world_pos:
        all_de = [we - drone_e for we, wn, *_ in weed_world_pos]
        all_dn = [wn - drone_n for we, wn, *_ in weed_world_pos]
        span = max(max(abs(v) for v in all_de + all_dn) * 2, 1.0)
        raw_scale = min(w, bh) * 0.70 / span
        scale = min(_MAP_MAX_SCALE * s, max(_MAP_MIN_SCALE * s, raw_scale))
    else:
        scale = _MAP_DEFAULT_SCALE * s

    mcx = x + w // 2
    mcy = by + bh // 2

    def w2s(de: float, dn: float) -> tuple[int, int]:
        return mcx + int(de * scale), mcy - int(dn * scale)

    metres_half = (min(w, bh) / 2) / scale
    for interval in (0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500):
        if metres_half / 3 <= interval:
            grid_m = interval
            break
    else:
        grid_m = 500

    g_range = int(metres_half / grid_m) + 2
    for gi in range(-g_range, g_range + 1):
        _, gy = w2s(0, gi * grid_m)
        if by <= gy <= by + bh:
            cv2.line(panel, (x, gy), (x + w, gy), (28, 34, 28), max(1, s))
        gx, _ = w2s(gi * grid_m, 0)
        if x <= gx <= x + w:
            cv2.line(panel, (gx, by), (gx, by + bh), (28, 34, 28), max(1, s))

    scale_lbl = f"{grid_m:.0f}m" if grid_m >= 1 else f"{grid_m * 100:.0f}cm"
    gfx.put_text(panel, scale_lbl, (x + 4 * s, by + bh - 4 * s), 0.28 * s, (45, 60, 45))

    nax, nay = x + w - 14 * s, by + 20 * s
    cv2.line(panel, (nax, nay + 8 * s), (nax, nay - 8 * s), (70, 70, 180), max(1, s))
    cv2.fillPoly(panel,
                 [np.array([(nax, nay - 8 * s),
                             (nax - 4 * s, nay),
                             (nax + 4 * s, nay)], np.int32)],
                 (70, 70, 180))
    gfx.put_text(panel, 'N', (nax - 4 * s, nay + 18 * s), 0.28 * s, (70, 70, 180))

    for item in weed_world_pos:
        we, wn, wid, active = item
        wx, wy = w2s(we - drone_e, wn - drone_n)
        if x <= wx <= x + w and by <= wy <= by + bh:
            col = (50, 200, 90) if active else (65, 65, 65)
            cv2.circle(panel, (wx, wy), 4 * s, col, -1)
            gfx.put_text(panel, f"W{wid}", (wx + 5 * s, wy + 4 * s), 0.26 * s, col)

    dr  = 9 * s
    tip = (mcx + int(dr * math.sin(yaw_rad)),       mcy - int(dr * math.cos(yaw_rad)))
    tl  = (mcx + int(dr * 0.6 * math.sin(yaw_rad + 2.4)),
           mcy - int(dr * 0.6 * math.cos(yaw_rad + 2.4)))
    tr  = (mcx + int(dr * 0.6 * math.sin(yaw_rad - 2.4)),
           mcy - int(dr * 0.6 * math.cos(yaw_rad - 2.4)))
    cv2.fillPoly(panel, [np.array([tip, tl, tr], np.int32)], (0, 220, 255))
    cv2.circle(panel, (mcx, mcy), 3 * s, (0, 220, 255), -1)


def _draw_program_panel(panel: np.ndarray, x: int, y: int, w: int,
                        armed: bool = False) -> None:
    global _arm_btn_rect
    _arm_btn_rect = (0, 0, 0, 0)   # no ARM button — arming is via pre-flight overlay only
    s = config.PR
    P = 10 * s
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), max(1, s))
    gfx.put_text(panel, 'PROGRAMS', (x + P, y + 18 * s), 0.40 * s, (95, 95, 95))
    cv2.line(panel, (x, y + 24 * s), (x + w, y + 24 * s), (48, 48, 48), max(1, s))

    sel = _ui_state['selected_prog']
    run = _ui_state['running_prog']
    for i, prog in enumerate(PROGRAMS):
        ry     = y + 28 * s + i * 26 * s
        is_sel = (i == sel)
        is_run = (i == run)
        if is_sel and not is_run:
            cv2.rectangle(panel,
                          (x + 2 * s, ry + s), (x + w - 2 * s, ry + 24 * s),
                          (44, 37, 16), -1)
        now    = time.monotonic()
        denied = (_ui_state['denied_prog'] == i and _ui_state['denied_until'] > now)
        cv2.circle(panel, (x + P + 4 * s, ry + 13 * s), 4 * s,
                   (50, 210, 50) if is_run else
                   (40, 40, 210) if denied  else
                   (52, 52, 52), -1)
        text_col = ((72, 72, 72) if is_run else
                    (100, 200, 255) if is_sel else
                    (162, 162, 162))
        gfx.put_text(panel, prog, (x + P + 18 * s, ry + 18 * s), 0.40 * s, text_col)

    sep_y    = y + 28 * s + len(PROGRAMS) * 26 * s + 4 * s
    cv2.line(panel, (x + 4 * s, sep_y), (x + w - 4 * s, sep_y), (52, 52, 52), max(1, s))
    by0, by1 = sep_y + 6 * s, sep_y + 30 * s

    def _btn(bx0, bx1, bg, border, label, fg, scale=0.40):
        cv2.rectangle(panel, (bx0, by0), (bx1, by1), bg, -1)
        cv2.rectangle(panel, (bx0, by0), (bx1, by1), border, max(1, s))
        (lw, lh), _ = gfx.size(label, scale * s)
        gfx.put_text(panel, label,
                     (bx0 + (bx1 - bx0 - lw) // 2, by0 + (by1 - by0 + lh) // 2),
                     scale * s, fg)

    def _gbtn(bx0, bx1, label, scale=0.40):
        _btn(bx0, bx1, (26, 30, 34), (45, 52, 58), label, (65, 78, 90), scale)

    if _ui_state['landing']:
        _btn(x + 8 * s, x + w - 8 * s,
             (18, 18, 100), (40, 40, 200), 'ABORT LAND', (80, 80, 235))
    elif _ui_state['stopped']:
        _btn(x + 8 * s,        x + 88 * s,      (18, 18, 120), (40, 40, 190), 'LAND',   (80, 80, 230))
        _btn(x + 92 * s,       x + 172 * s,     (18, 80, 18),  (40, 160, 40), 'RESUME', (80, 220, 80), 0.38)
        _gbtn(x + w - 88 * s,  x + w - 8 * s,   'SEND')
    else:
        _btn(x + 8 * s,        x + 88 * s,      (18, 18, 120), (40, 40, 190),  'STOP',   (80, 80, 230))
        _btn(x + w - 88 * s,   x + w - 8 * s,   (34, 54, 90),  (72, 108, 160), 'SEND',   (172, 210, 255))

    # ── Motor test (bench bring-up — PROPS OFF) ────────────────────────────────
    # One M1..M4 button row. Each sends MAV_CMD_DO_MOTOR_TEST; the firmware spins
    # that motor for 2 s and only while disarmed + Idle, so it is safe on the bench.
    mt_y0 = y + _MT_BTN_Y0 * s
    mt_y1 = y + _MT_BTN_Y1 * s
    gfx.put_text(panel, 'MOTOR TEST', (x + 8 * s, mt_y0 - 5 * s), 0.32 * s, (120, 110, 70))
    (pw, _), _ = gfx.size('PROPS OFF', 0.32 * s)
    gfx.put_text(panel, 'PROPS OFF', (x + w - pw - 8 * s, mt_y0 - 5 * s), 0.32 * s, (90, 120, 175))
    for i, (bx0, bx1) in enumerate(_motor_test_btn_xranges(w, s)):
        cv2.rectangle(panel, (x + bx0, mt_y0), (x + bx1, mt_y1), (40, 36, 20), -1)
        cv2.rectangle(panel, (x + bx0, mt_y0), (x + bx1, mt_y1), (95, 80, 38), max(1, s))
        lbl = f'M{i + 1}'
        (lw2, lh2), _ = gfx.size(lbl, 0.40 * s)
        gfx.put_text(panel, lbl,
                     (x + bx0 + (bx1 - bx0 - lw2) // 2, mt_y0 + (mt_y1 - mt_y0 + lh2) // 2),
                     0.40 * s, (210, 175, 95))


def _handle_overlay_click(lx: int, ly: int) -> None:
    """Dispatch a click at local overlay coords (logical pixels)."""
    title_h = 28
    if ly < title_h:
        return
    row_i = (ly - title_h) // 26
    if row_i < len(PROGRAMS):
        _ui_state['selected_prog'] = row_i
        return
    # Motor-test buttons (M1..M4), below the SEND/STOP row. PROPS OFF — the
    # firmware only honours this while disarmed and on the ground (Idle).
    if _MT_BTN_Y0 <= ly <= _MT_BTN_Y1:
        for i, (bx0, bx1) in enumerate(_motor_test_btn_xranges(config.STATS_W // 2, 1)):
            if bx0 <= lx <= bx1:
                send_mavlink_command(209, float(i + 1), MOTOR_TEST_THROTTLE)
                print(f"Motor test: M{i + 1} @ {int(MOTOR_TEST_THROTTLE * 100)}% for 2 s "
                      f"— PROPS OFF (firmware requires disarmed + Idle)", flush=True)
                return
        return
    # SEND / STOP / LAND / RESUME button row. Tight hit-boxes matching the drawn
    # buttons (see _draw_program_panel) so clicks in the surrounding padding do
    # nothing instead of firing a command.
    if not (_PROG_BTN_Y0 <= ly <= _PROG_BTN_Y1):
        return
    panel_w      = config.STATS_W // 2
    on_stop_land = 8 <= lx <= 88                       # STOP, or LAND when stopped
    on_resume    = 92 <= lx <= 172                     # RESUME (only when stopped)
    on_send      = panel_w - 88 <= lx <= panel_w - 8   # SEND
    if _ui_state['landing']:
        # Single full-width ABORT LAND button.
        if 8 <= lx <= panel_w - 8:
            _ui_state['landing'] = False
            _ui_state['stopped'] = False
            send_mavlink_command(20)
            print("Landing aborted — RTH commanded", flush=True)
        return
    if _ui_state['stopped']:
        if on_stop_land:
            _ui_state['landing'] = True
            _ui_state['running_prog'] = None
            send_mavlink_command(21)
        elif on_resume:
            _ui_state['stopped'] = False
            print("Resumed", flush=True)
    else:
        if on_stop_land:
            _ui_state['stopped'] = True
            print("STOP — choose LAND or RESUME", flush=True)
        elif on_send:
            _ui_state['running_prog'] = _ui_state['selected_prog']
            prog = _ui_state['running_prog']
            cmd, p1, p2 = _PROG_COMMANDS.get(prog, (0, 0.0, 0.0))
            if cmd:
                send_mavlink_command(cmd, p1, p2)
                _ui_state['pending_ack_prog'] = prog
            print(f"Program: {PROGRAMS[prog]}", flush=True)


def draw_stats_panel(h: int, tracker=None) -> np.ndarray:
    """Return a physical-pixel stats panel. h is in logical pixels."""
    s   = config.PR
    W   = config.STATS_W * s
    H   = h * s
    COL = W // 2
    panel = np.full((H, W, 3), (26, 26, 26), dtype=np.uint8)
    cv2.line(panel, (0, 0), (0, H - 1), (65, 65, 65), max(1, s))

    gfx.begin()   # clear text queue for this frame

    with _mav_lock:
        lat            = _mav_state['lat']
        lon            = _mav_state['lon']
        alt            = _mav_state['alt']
        yaw            = _mav_state['yaw']
        roll           = _mav_state['roll']
        pitch          = _mav_state['pitch']
        vx             = _mav_state['vx']
        vy             = _mav_state['vy']
        vz             = _mav_state['vz']
        batt           = _mav_state['battery_pct']
        armed          = _mav_state['armed']
        last_t         = _mav_state['last_msg_t']
        voltage_mv     = _mav_state['voltage_mv']
        current_ca     = _mav_state['current_ca']
        mah_con        = _mav_state['mah_consumed']
        time_rem       = _mav_state['time_remaining_s']
        motor_pwm      = list(_mav_state['motor_pwm'])
        origin         = _mav_state['origin']
        flight_mode_id = _mav_state['flight_mode_id']
        flight_state   = _mav_state['flight_state']
        payload_flags  = _mav_state['payload_flags']
        gps_fix_type   = _mav_state['gps_fix_type']
        throttle_pct   = _mav_state['throttle_pct']
        sensors_health = _mav_state['sensors_health']
        last_ack       = _mav_state['last_ack']
        if last_ack is not None:
            _mav_state['last_ack'] = None

    if last_ack is not None:
        ack_cmd, ack_result = last_ack
        pending = _ui_state['pending_ack_prog']
        if pending is not None and _PROG_COMMANDS.get(pending, (0,))[0] == ack_cmd:
            _ui_state['pending_ack_prog'] = None
            if ack_result != 0:
                _ui_state['denied_prog']  = pending
                _ui_state['denied_until'] = time.monotonic() + 3.0

    linked = last_t is not None and (time.monotonic() - last_t) < 2.0
    spd    = math.hypot(vx, vy)
    hdg    = math.degrees(yaw) % 360
    climb  = -vz

    # ── Header ───────────────────────────────────────────────────────────────
    cv2.rectangle(panel, (s, 0), (W, 36 * s), (40, 44, 50), -1)
    (tw, _), _ = gfx.size('FLIGHT COMPUTER', 0.46 * s)
    gfx.put_text(panel, 'FLIGHT COMPUTER', (W // 2 - tw // 2, 23 * s), 0.46 * s, (185, 192, 200))
    cv2.line(panel, (0, 36 * s), (W, 36 * s), (60, 60, 60), max(1, s))

    # ── Status row ───────────────────────────────────────────────────────────
    sy = 57 * s
    dot_col = (50, 210, 50) if linked else (60, 60, 200)
    cv2.circle(panel, (13 * s, sy - 5 * s), 5 * s, dot_col, -1)
    gfx.put_text(panel, 'LINKED' if linked else 'NO LINK', (24 * s, sy), 0.38 * s, dot_col)
    arm_lbl = 'ARMED' if armed else 'DISARMED'
    arm_col = (40, 230, 40) if armed else (75, 75, 75)
    (aw, _), _ = gfx.size(arm_lbl, 0.38 * s)
    gfx.put_text(panel, arm_lbl, (W - aw - 7 * s, sy), 0.38 * s, arm_col)
    if linked:
        state_lbl  = _FLIGHT_STATES.get(flight_state, f"ST{flight_state}")
        mode_lbl   = _FLIGHT_MODES.get(flight_mode_id, f"M{flight_mode_id}")
        center_lbl = (f"{state_lbl}\xb7{mode_lbl}"
                      if flight_state in (3, 4) else state_lbl)
        state_col  = _STATE_COLORS.get(flight_state, (150, 150, 150))
        (cw, _), _ = gfx.size(center_lbl, 0.38 * s)
        gfx.put_text(panel, center_lbl, (W // 2 - cw // 2, sy), 0.38 * s, state_col)

    CY = sy + 9 * s
    cv2.line(panel, (0, CY), (W, CY), (48, 48, 48), max(1, s))
    cv2.line(panel, (COL, CY), (COL, H - 1), (50, 50, 50), max(1, s))

    # ── Left column ──────────────────────────────────────────────────────────
    L          = 0
    BATT_NAV_H = 295 * s
    gauge_area = max(180 * s, H - CY - BATT_NAV_H)
    row_h      = gauge_area // 3

    ah_r  = max(20 * s, min(row_h // 2 - 18 * s, COL // 2 - 24 * s))
    ah_cx = L + COL // 2
    ah_cy = CY + row_h // 2
    _draw_artificial_horizon(panel, ah_cx, ah_cy,
                             ah_r, roll if linked else 0.0, pitch if linked else 0.0)
    rp_y = ah_cy + ah_r + 14 * s
    gfx.put_text(panel, f"R{math.degrees(roll):+.1f}",
                 (L + 8 * s, rp_y), 0.34 * s, (140, 140, 140))
    ps = f"P{math.degrees(pitch):+.1f}"
    (pw, _), _ = gfx.size(ps, 0.34 * s)
    gfx.put_text(panel, ps, (L + COL - pw - 8 * s, rp_y), 0.34 * s, (140, 140, 140))
    sep1 = CY + row_h
    cv2.line(panel, (L, sep1), (L + COL, sep1), (48, 48, 48), max(1, s))

    gr1  = max(14 * s, min(row_h // 2 - 24 * s, COL // 4 - 12 * s))
    g1cx = L + COL // 4
    g1cy = sep1 + row_h // 2
    g2cx = L + 3 * COL // 4
    _draw_compass(panel, g1cx, g1cy, gr1, hdg if linked else 0.0)
    _draw_arc_gauge(panel, g2cx, g1cy, gr1,
                    alt if linked else 0.0, 0.0, 80.0, 'ALT', 'm', (80, 200, 80))
    sep2 = CY + 2 * row_h
    cv2.line(panel, (L, sep2), (L + COL, sep2), (48, 48, 48), max(1, s))

    gr2  = gr1
    g3cx = L + COL // 4
    g3cy = sep2 + row_h // 2
    g4cx = L + 3 * COL // 4
    _draw_arc_gauge(panel, g3cx, g3cy, gr2,
                    spd if linked else 0.0, 0.0, 15.0,
                    'SPEED', 'm/s', (80, 155, 230), warn_frac=0.73)
    _draw_vspeed(panel, g4cx, g3cy, gr2, climb if linked else 0.0)

    power_y = H - BATT_NAV_H
    cv2.line(panel, (L, power_y - 4 * s), (L + COL, power_y - 4 * s), (48, 48, 48), max(1, s))
    _draw_battery_panel(panel, L, power_y, COL,
                        voltage_mv if linked else -1,
                        current_ca if linked else -1,
                        batt,
                        mah_con    if linked else -1,
                        time_rem   if linked else 0)
    _draw_distance_home(panel, L, power_y + 155 * s, COL, lat, lon, origin)

    # ── Right column ─────────────────────────────────────────────────────────
    R          = COL
    MOTORS_H   = 260 * s
    payloads_h = (24 + len(PAYLOAD_NAMES) * 24 + 20) * s
    map_h      = max(0, H - CY - 4 * s - MOTORS_H - payloads_h - OVERLAY_H * s - 4 * s)
    map_y      = CY + 4 * s + MOTORS_H

    active_pwm   = [p for p in motor_pwm if p > 1050]
    motor_spread = (max(active_pwm) - min(active_pwm)) if len(active_pwm) >= 2 else 0

    statuses = [
        ('CTRL',   ('FAULT' if flight_state == 5
                    else 'CHECK' if flight_state == 1
                    else 'OK')  if linked else '--'),
        ('LINK',   'OK'   if linked else 'FAULT'),
        (None, None),
        ('GPS',    ('OK'    if gps_fix_type >= 3
                    else 'CHECK' if gps_fix_type >= 1
                    else 'FAULT') if linked else '--'),
        ('IMU',    ('OK'    if sensors_health & 0x03 == 0x03
                    else 'FAULT' if sensors_health != 0
                    else '--')   if linked else '--'),
        ('BARO',   ('OK'    if sensors_health & 0x08
                    else 'FAULT' if sensors_health != 0
                    else '--')   if linked else '--'),
        ('BATT',   '--'   if batt < 0
                   else 'OK'    if batt > 30
                   else 'CHECK' if batt > 15
                   else 'FAULT'),
        ('MOTORS', ('CHECK' if armed and motor_spread > 300 else 'OK') if linked else '--'),
    ]

    half = COL // 2
    _draw_status_pane(panel, R,        CY + 4 * s, half,       MOTORS_H, statuses)
    _draw_motors(     panel, R + half, CY + 4 * s, COL - half, MOTORS_H,
                 motor_pwm if linked else [0, 0, 0, 0], armed,
                 throttle_pct if linked else 0)

    if map_h > 40 * s:
        weed_world_pos = []
        if tracker is not None:
            for wid, w in tracker._named.items():
                wp = w.get('world_pos')
                if wp is not None:
                    weed_world_pos.append((wp[0], wp[1], wid, w['lost_frames'] == 0))
        _draw_weed_map(panel, R, map_y, COL, map_h,
                       lat, lon, yaw, origin, weed_world_pos, linked)

    global _payload_panel_rect
    _payload_panel_rect = (R // s, (map_y + map_h) // s, COL // s, payloads_h // s)
    _draw_payloads(panel, R, map_y + map_h, COL, payload_flags, linked)
    _draw_program_panel(panel, R, H - OVERLAY_H * s, COL, armed)

    gfx.flush(panel)   # render all queued text in one PIL pass
    return panel
