import math
import time

import cv2
import numpy as np

import config
from config import PROGRAMS, PAYLOAD_NAMES, OVERLAY_W, OVERLAY_H
from mavlink import _mav_lock, _mav_state

_FLIGHT_MODES = {
    0: 'STAB', 1: 'ALTH', 2: 'POSH', 3: 'AUTO', 4: 'RTH', 5: 'LAND',
}
_FLIGHT_STATES = {
    0: 'IDLE', 1: 'ARMING', 2: 'ARMED', 3: 'FLYING', 4: 'LANDING', 5: 'FAULT',
}
_STATE_COLORS = {
    0: (90,  90,  90),   # IDLE    — grey
    1: (30, 160, 200),   # ARMING  — amber
    2: (30, 160, 200),   # ARMED   — amber
    3: (80, 200,  80),   # FLYING  — green
    4: (40, 140, 220),   # LANDING — blue
    5: (40,  40, 210),   # FAULT   — red
}

_ui_state: dict = {'selected_prog': 0, 'running_prog': None}


def _draw_artificial_horizon(panel: np.ndarray, cx: int, cy: int, r: int,
                              roll_rad: float, pitch_rad: float) -> None:
    sky_bgr = (150, 90, 50)
    gnd_bgr = (35, 100, 160)

    pixels_per_rad = r / math.radians(35)
    pitch_px = max(-r, min(r, int(pitch_rad * pixels_per_rad)))

    # sky_dir: unit vector toward sky in image coords (y-down)
    # Rotating (0, -1) by roll: sky_x = sin(roll), sky_y = -cos(roll)
    sky_x =  math.sin(roll_rad)
    sky_y = -math.cos(roll_rad)
    hdx   =  math.cos(roll_rad)   # horizon tangent
    hdy   =  math.sin(roll_rad)

    # Horizon anchor: positive pitch (nose up) shifts horizon downward
    hx = cx - int(pitch_px * sky_x)
    hy = cy - int(pitch_px * sky_y)
    ext = r + 6

    l  = (int(hx - ext * hdx), int(hy - ext * hdy))
    ri = (int(hx + ext * hdx), int(hy + ext * hdy))
    tl = (int(l[0]  + ext * sky_x), int(l[1]  + ext * sky_y))
    tr = (int(ri[0] + ext * sky_x), int(ri[1] + ext * sky_y))

    cv2.circle(panel, (cx, cy), r, gnd_bgr, -1)
    mask = np.zeros(panel.shape[:2], dtype=np.uint8)
    cv2.circle(mask, (cx, cy), r, 255, -1)
    tmp = panel.copy()
    cv2.fillPoly(tmp, [np.array([l, ri, tr, tl], np.int32)], sky_bgr)
    panel[mask == 255] = tmp[mask == 255]

    # Pitch ladder
    for deg in [-20, -10, 10, 20]:
        off = math.radians(deg) * pixels_per_rad
        px  = int(hx - off * sky_x)
        py  = int(hy - off * sky_y)
        hw  = r // 4 if abs(deg) == 20 else r // 6
        cv2.line(panel,
                 (int(px - hw * hdx), int(py - hw * hdy)),
                 (int(px + hw * hdx), int(py + hw * hdy)),
                 (200, 200, 200), 1)

    cv2.line(panel, l, ri, (240, 240, 240), 2)

    # Roll arc + pointer triangle
    roll_deg = math.degrees(roll_rad)
    cv2.ellipse(panel, (cx, cy), (r - 5, r - 5), 0, -140, -40, (160, 160, 160), 1)
    ta  = math.radians(-90 - roll_deg)
    tip = (int(cx + (r - 5) * math.cos(ta)), int(cy + (r - 5) * math.sin(ta)))
    tl2 = (int(cx + (r + 3) * math.cos(ta - 0.15)), int(cy + (r + 3) * math.sin(ta - 0.15)))
    tr2 = (int(cx + (r + 3) * math.cos(ta + 0.15)), int(cy + (r + 3) * math.sin(ta + 0.15)))
    cv2.fillPoly(panel, [np.array([tip, tl2, tr2], np.int32)], (0, 220, 255))

    # Fixed aircraft symbol
    hw = r // 3
    cv2.line(panel, (cx - hw, cy), (cx - hw // 3, cy), (0, 220, 255), 2)
    cv2.line(panel, (cx + hw // 3, cy), (cx + hw, cy), (0, 220, 255), 2)
    cv2.circle(panel, (cx, cy), 3, (0, 220, 255), -1)
    cv2.circle(panel, (cx, cy), r, (90, 90, 90), 2)


def _draw_compass(panel: np.ndarray, cx: int, cy: int, r: int,
                  heading_deg: float) -> None:
    cv2.circle(panel, (cx, cy), r, (45, 45, 45), -1)
    cv2.circle(panel, (cx, cy), r, (90, 90, 90), 1)

    for deg in range(0, 360, 45):
        a     = math.radians(deg - heading_deg - 90)
        inner = r - (9 if deg % 90 == 0 else 5)
        p1 = (int(cx + inner * math.cos(a)), int(cy + inner * math.sin(a)))
        p2 = (int(cx + (r - 1) * math.cos(a)), int(cy + (r - 1) * math.sin(a)))
        cv2.line(panel, p1, p2, (130, 130, 130), 1)

    for deg, lbl in ((0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')):
        a  = math.radians(deg - heading_deg - 90)
        lx = int(cx + (r - 16) * math.cos(a)) - 4
        ly = int(cy + (r - 16) * math.sin(a)) + 4
        col = (60, 60, 220) if lbl == 'N' else (190, 190, 190)
        cv2.putText(panel, lbl, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv2.LINE_AA)

    # Fixed heading pointer at top
    tip = (cx, cy - r + 3)
    cv2.fillPoly(panel,
                 [np.array([tip, (cx - 5, cy - r + 13), (cx + 5, cy - r + 13)], np.int32)],
                 (60, 180, 255))
    hdg_s = f"{int(heading_deg) % 360:03d}°"
    (tw, _), _ = cv2.getTextSize(hdg_s, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    cv2.putText(panel, hdg_s, (cx - tw // 2, cy + 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)


def _draw_arc_gauge(panel: np.ndarray, cx: int, cy: int, r: int,
                    value: float, vmin: float, vmax: float,
                    label: str, unit: str, arc_color: tuple,
                    warn_frac: float | None = None) -> None:
    """270° sweep gauge: lower-left (135°) → top → lower-right (405°=45°)."""
    cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 405, (55, 55, 55), 6)

    frac  = max(0.0, min(1.0, (value - vmin) / (vmax - vmin)))
    color = (40, 100, 230) if (warn_frac and frac >= warn_frac) else arc_color
    if frac > 0:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 135 + frac * 270, color, 5)

    na   = math.radians(135 + frac * 270)
    ntip = (int(cx + (r - 7) * math.cos(na)), int(cy + (r - 7) * math.sin(na)))
    cv2.line(panel, (cx, cy), ntip, (220, 220, 220), 2)
    cv2.circle(panel, (cx, cy), 4, (170, 170, 170), -1)

    val_s = (f"{value:.0f}" if abs(value) >= 10 else f"{value:.1f}") + unit
    (tw, _), _ = cv2.getTextSize(val_s, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)
    cv2.putText(panel, val_s, (cx - tw // 2, cy + r + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (215, 215, 215), 1, cv2.LINE_AA)
    (lw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.32, 1)
    cv2.putText(panel, label, (cx - lw // 2, cy + r + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (105, 105, 105), 1, cv2.LINE_AA)


def _draw_vspeed(panel: np.ndarray, cx: int, cy: int, r: int,
                 climb_ms: float) -> None:
    """Vertical speed indicator: ±5 m/s, zero at top of arc (270°)."""
    MAX_MS = 5.0
    cv2.ellipse(panel, (cx, cy), (r, r), 0, 135, 405, (55, 55, 55), 6)
    cv2.line(panel, (cx, cy - r + 3), (cx, cy - r + 11), (160, 160, 160), 2)

    frac   = max(-1.0, min(1.0, climb_ms / MAX_MS))
    na_deg = 270 + frac * 135   # CW from zero (top) toward lower-right for climb
    na     = math.radians(na_deg)
    arc_col = (60, 200, 60) if frac >= 0 else (60, 60, 210)
    if frac >= 0:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, 270, na_deg, arc_col, 5)
    else:
        cv2.ellipse(panel, (cx, cy), (r, r), 0, na_deg, 270, arc_col, 5)

    ntip = (int(cx + (r - 7) * math.cos(na)), int(cy + (r - 7) * math.sin(na)))
    cv2.line(panel, (cx, cy), ntip, (220, 220, 220), 2)
    cv2.circle(panel, (cx, cy), 4, (170, 170, 170), -1)

    sign  = '+' if climb_ms >= 0 else ''
    val_s = f"{sign}{climb_ms:.1f}m/s"
    (tw, _), _ = cv2.getTextSize(val_s, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(panel, val_s, (cx - tw // 2, cy + r + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
    (lw, _), _ = cv2.getTextSize('V.SPEED', cv2.FONT_HERSHEY_SIMPLEX, 0.32, 1)
    cv2.putText(panel, 'V.SPEED', (cx - lw // 2, cy + r + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (105, 105, 105), 1, cv2.LINE_AA)


def _draw_battery(panel: np.ndarray, x: int, y: int, w: int, h: int,
                  pct: int) -> None:
    cv2.rectangle(panel, (x, y), (x + w, y + h), (70, 70, 70), 1)
    if pct >= 0:
        fill_w = max(1, int(w * pct / 100))
        green  = min(255, int(510 * pct / 100))
        red    = min(255, int(510 * (100 - pct) / 100))
        cv2.rectangle(panel, (x + 1, y + 1), (x + fill_w, y + h - 1), (0, green, red), -1)
    pct_s = f"{pct}%" if pct >= 0 else "--"
    cv2.putText(panel, pct_s, (x + w + 6, y + h - 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (170, 170, 170), 1, cv2.LINE_AA)


def _draw_battery_panel(panel: np.ndarray, x: int, y: int, w: int,
                        voltage_mv: int, current_ca: int, battery_pct: int,
                        mah_consumed: int, time_remaining_s: int) -> None:
    """Voltage, current, charge bar, time remaining, consumed mAh."""
    P = 10
    cv2.putText(panel, 'POWER', (x + P, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(panel, (x, y + 22), (x + w, y + 22), (48, 48, 48), 1)

    cy = y + 38
    v_s = f"{voltage_mv / 1000:.1f}V" if voltage_mv > 0  else "--V"
    a_s = f"{current_ca / 100:.1f}A"  if current_ca >= 0 else "--A"
    cv2.putText(panel, v_s, (x + P, cy + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.80, (200, 210, 100), 1, cv2.LINE_AA)
    (aw, _), _ = cv2.getTextSize(a_s, cv2.FONT_HERSHEY_SIMPLEX, 0.80, 1)
    cv2.putText(panel, a_s, (x + w - aw - P, cy + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.80, (100, 180, 220), 1, cv2.LINE_AA)
    cv2.putText(panel, 'VOLT', (x + P, cy + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (70, 70, 70), 1, cv2.LINE_AA)
    cv2.putText(panel, 'CURR', (x + w - 30 - P, cy + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (70, 70, 70), 1, cv2.LINE_AA)
    cy += 44

    cv2.putText(panel, 'CHARGE', (x + P, cy + 11),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)
    _draw_battery(panel, x + 65, cy, w - 75 - P, 13, battery_pct)
    cy += 24

    if time_remaining_s > 0:
        mins, secs = divmod(time_remaining_s, 60)
        t_s = f"{mins}:{secs:02d}"
    else:
        t_s = "--:--"
    cv2.putText(panel, 'BATT TIME', (x + P, cy + 11),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)
    (tw, _), _ = cv2.getTextSize(t_s, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
    cv2.putText(panel, t_s, (x + w - tw - P, cy + 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1, cv2.LINE_AA)
    cy += 24

    mah_s = f"{mah_consumed:.0f} mAh" if mah_consumed >= 0 else "-- mAh"
    cv2.putText(panel, 'CONSUMED', (x + P, cy + 11),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)
    (mw, _), _ = cv2.getTextSize(mah_s, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    cv2.putText(panel, mah_s, (x + w - mw - P, cy + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1, cv2.LINE_AA)


def _draw_distance_home(panel: np.ndarray, x: int, y: int, w: int,
                        lat, lon, origin, yaw: float) -> None:
    """Navigation section: distance/bearing from launch point + GPS coords."""
    P = 10
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), 1)
    cv2.putText(panel, 'NAVIGATION', (x + P, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(panel, (x, y + 24), (x + w, y + 24), (48, 48, 48), 1)

    if origin is not None and lat is not None:
        R     = 6_371_000.0
        lat0, lon0 = origin
        east  = math.radians(lon  - lon0) * R * math.cos(math.radians(lat0))
        north = math.radians(lat  - lat0) * R
        dist  = math.hypot(east, north)
        bear  = (math.degrees(math.atan2(east, north)) + 360) % 360
        dist_s, bear_s = f"{dist:.1f} m", f"{bear:.0f}°"
    else:
        dist_s, bear_s = "-- m", "--°"

    cy = y + 44
    cv2.putText(panel, 'DIST HOME', (x + P, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)
    (dw, _), _ = cv2.getTextSize(dist_s, cv2.FONT_HERSHEY_SIMPLEX, 0.68, 1)
    cv2.putText(panel, dist_s, (x + w - dw - P, cy + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.68, (215, 195, 130), 1, cv2.LINE_AA)
    cy += 30
    cv2.putText(panel, 'BEARING', (x + P, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (95, 95, 95), 1, cv2.LINE_AA)
    (bw, _), _ = cv2.getTextSize(bear_s, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.putText(panel, bear_s, (x + w - bw - P, cy + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (185, 185, 185), 1, cv2.LINE_AA)
    cy += 26
    cv2.putText(panel, f"LAT  {lat:.5f}" if lat is not None else "LAT  --",
                (x + P, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (128, 128, 128), 1, cv2.LINE_AA)
    cy += 18
    cv2.putText(panel, f"LON  {lon:.5f}" if lon is not None else "LON  --",
                (x + P, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (128, 128, 128), 1, cv2.LINE_AA)


def _draw_motors(panel: np.ndarray, x: int, y: int, w: int, h: int,
                 motor_pwm: list, armed: bool) -> None:
    """X-frame quadrotor motor diagram; ring colour = green/yellow/red by PWM."""
    P = 10
    cv2.putText(panel, 'MOTORS', (x + P, y + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(panel, (x, y + 22), (x + w, y + 22), (48, 48, 48), 1)

    dh  = h - 22
    dcx = x + w // 2
    dcy = y + 22 + dh // 2
    arm = min(w // 2, dh // 2) - 20

    positions = [
        (dcx - arm, dcy - arm),   # M1 FL
        (dcx + arm, dcy - arm),   # M2 FR
        (dcx - arm, dcy + arm),   # M3 RL
        (dcx + arm, dcy + arm),   # M4 RR
    ]
    for mx, my in positions:
        cv2.line(panel, (dcx, dcy), (mx, my), (58, 58, 58), 2)

    cv2.circle(panel, (dcx, dcy), 8, (88, 88, 88), -1)
    cv2.circle(panel, (dcx, dcy), 8, (115, 115, 115), 1)
    cv2.fillPoly(panel,
                 [np.array([(dcx - 4, dcy - 8), (dcx + 4, dcy - 8), (dcx, dcy - 15)],
                            np.int32)], (90, 170, 255))

    mr = 14
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
        cv2.circle(panel, (mx, my), mr, ring, 2)
        lbl = f"M{i + 1}"
        (lw, lh), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.30, 1)
        cv2.putText(panel, lbl, (mx - lw // 2, my + lh // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.30, (158, 158, 158), 1, cv2.LINE_AA)
        pwm_s = str(pwm) if pwm > 0 else "--"
        (pw, _), _ = cv2.getTextSize(pwm_s, cv2.FONT_HERSHEY_SIMPLEX, 0.27, 1)
        lbl_y = my + mr + 11 if my >= dcy else my - mr - 3
        cv2.putText(panel, pwm_s, (mx - pw // 2, lbl_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.27, (88, 88, 88), 1, cv2.LINE_AA)


def _draw_payloads(panel: np.ndarray, x: int, y: int, w: int,
                   payload_flags: int, linked: bool) -> None:
    """Payload health panel — green dot = connected, dark-red dot = not detected."""
    P = 10
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), 1)
    cv2.putText(panel, 'PAYLOADS', (x + P, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(panel, (x, y + 24), (x + w, y + 24), (48, 48, 48), 1)
    ry = y + 44
    for bit, name in PAYLOAD_NAMES.items():
        present = linked and bool(payload_flags & (1 << bit))
        dot_col = (50, 200, 100) if present else (50, 40, 80)
        status_col = (130, 210, 130) if present else (90, 65, 100)
        status_lbl = 'OK' if present else '--'
        cv2.circle(panel, (x + P + 4, ry - 4), 4, dot_col, -1)
        cv2.putText(panel, name, (x + P + 14, ry),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (162, 162, 162), 1, cv2.LINE_AA)
        (sw, _), _ = cv2.getTextSize(status_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)
        cv2.putText(panel, status_lbl, (x + w - sw - P, ry),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, status_col, 1, cv2.LINE_AA)
        ry += 24


def _draw_program_panel(panel: np.ndarray, x: int, y: int, w: int) -> None:
    """Program selector drawn directly in the stats panel (right column, bottom)."""
    P = 10
    cv2.line(panel, (x, y), (x + w, y), (48, 48, 48), 1)
    cv2.putText(panel, 'PROGRAMS', (x + P, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (95, 95, 95), 1, cv2.LINE_AA)
    cv2.line(panel, (x, y + 24), (x + w, y + 24), (48, 48, 48), 1)

    sel = _ui_state['selected_prog']
    run = _ui_state['running_prog']
    for i, prog in enumerate(PROGRAMS):
        ry     = y + 28 + i * 26
        is_sel = (i == sel)
        is_run = (i == run)
        if is_sel and not is_run:
            cv2.rectangle(panel, (x + 2, ry + 1), (x + w - 2, ry + 24), (44, 37, 16), -1)
        cv2.circle(panel, (x + P + 4, ry + 13), 4,
                   (50, 210, 50) if is_run else (52, 52, 52), -1)
        if is_run:
            text_col = (72, 72, 72)
        elif is_sel:
            text_col = (100, 200, 255)
        else:
            text_col = (162, 162, 162)
        cv2.putText(panel, prog, (x + P + 18, ry + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, text_col, 1, cv2.LINE_AA)

    sep_y = y + 28 + len(PROGRAMS) * 26 + 4
    cv2.line(panel, (x + 4, sep_y), (x + w - 4, sep_y), (52, 52, 52), 1)
    bx0, bx1 = x + w - 88, x + w - 8
    by0, by1 = sep_y + 6, sep_y + 30
    cv2.rectangle(panel, (bx0, by0), (bx1, by1), (34, 54, 90), -1)
    cv2.rectangle(panel, (bx0, by0), (bx1, by1), (72, 108, 160), 1)
    cv2.putText(panel, 'SEND', (bx0 + 14, by0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (172, 210, 255), 1, cv2.LINE_AA)


def _draw_program_overlay(frame: np.ndarray, x: int, y: int) -> None:
    """Semi-transparent program selector box overlaid on the camera feed."""
    w, h = OVERLAY_W, OVERLAY_H
    if y < 0 or x < 0 or y + h > frame.shape[0] or x + w > frame.shape[1]:
        return
    roi = frame[y:y + h, x:x + w]
    cv2.addWeighted(np.full_like(roi, (16, 16, 16)), 0.86, roi, 0.14, 0, roi)
    frame[y:y + h, x:x + w] = roi
    cv2.rectangle(frame, (x, y), (x + w, y + h), (72, 72, 72), 1)
    cv2.rectangle(frame, (x + 1, y + 1), (x + w - 1, y + 27), (38, 44, 50), -1)
    cv2.putText(frame, 'PROGRAMS', (x + 9, y + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (178, 188, 196), 1, cv2.LINE_AA)
    cv2.line(frame, (x, y + 27), (x + w, y + 27), (58, 58, 58), 1)

    sel = _ui_state['selected_prog']
    run = _ui_state['running_prog']
    for i, prog in enumerate(PROGRAMS):
        ry     = y + 28 + i * 26
        is_sel = (i == sel)
        is_run = (i == run)
        if is_sel and not is_run:
            cv2.rectangle(frame, (x + 2, ry + 1), (x + w - 2, ry + 24), (44, 37, 16), -1)
        cv2.circle(frame, (x + 13, ry + 13), 4,
                   (50, 210, 50) if is_run else (52, 52, 52), -1)
        if is_run:
            text_col = (72, 72, 72)
        elif is_sel:
            text_col = (100, 200, 255)
        else:
            text_col = (162, 162, 162)
        cv2.putText(frame, prog, (x + 25, ry + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, text_col, 1, cv2.LINE_AA)

    sep_y  = y + 28 + len(PROGRAMS) * 26 + 4
    cv2.line(frame, (x + 4, sep_y), (x + w - 4, sep_y), (52, 52, 52), 1)
    bx0, bx1 = x + w - 88, x + w - 4
    by0, by1 = sep_y + 6, sep_y + 30
    cv2.rectangle(frame, (bx0, by0), (bx1, by1), (34, 54, 90), -1)
    cv2.rectangle(frame, (bx0, by0), (bx1, by1), (72, 108, 160), 1)
    cv2.putText(frame, 'SEND', (bx0 + 14, by0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (172, 210, 255), 1, cv2.LINE_AA)


def _handle_overlay_click(lx: int, ly: int) -> None:
    """Dispatch a click at local overlay coords to select a program or send."""
    title_h = 28
    if ly < title_h:
        return
    row_i = (ly - title_h) // 26
    if row_i < len(PROGRAMS):
        _ui_state['selected_prog'] = row_i
        return
    sep_y = title_h + len(PROGRAMS) * 26 + 4
    if ly >= sep_y + 6:
        _ui_state['running_prog'] = _ui_state['selected_prog']
        print(f"Instruction sent: {PROGRAMS[_ui_state['running_prog']]}", flush=True)


def draw_stats_panel(h: int) -> np.ndarray:
    """Return a STATS_W × h flight computer dashboard — 2-column layout."""
    COL = config.STATS_W // 2
    panel = np.full((h, config.STATS_W, 3), (26, 26, 26), dtype=np.uint8)
    cv2.line(panel, (0, 0), (0, h - 1), (65, 65, 65), 1)

    with _mav_lock:
        lat           = _mav_state['lat']
        lon           = _mav_state['lon']
        alt           = _mav_state['alt']
        yaw           = _mav_state['yaw']
        roll          = _mav_state['roll']
        pitch         = _mav_state['pitch']
        vx            = _mav_state['vx']
        vy            = _mav_state['vy']
        vz            = _mav_state['vz']
        batt          = _mav_state['battery_pct']
        armed         = _mav_state['armed']
        last_t        = _mav_state['last_msg_t']
        voltage_mv    = _mav_state['voltage_mv']
        current_ca    = _mav_state['current_ca']
        mah_con       = _mav_state['mah_consumed']
        time_rem      = _mav_state['time_remaining_s']
        motor_pwm     = list(_mav_state['motor_pwm'])
        origin        = _mav_state['origin']
        flight_mode_id = _mav_state['flight_mode_id']
        flight_state   = _mav_state['flight_state']
        payload_flags  = _mav_state['payload_flags']

    linked = last_t is not None and (time.monotonic() - last_t) < 2.0
    spd    = math.hypot(vx, vy)
    hdg    = math.degrees(yaw) % 360
    climb  = -vz

    # ── Header (full width) ───────────────────────────────────────────────────
    cv2.rectangle(panel, (1, 0), (config.STATS_W, 36), (40, 44, 50), -1)
    cv2.putText(panel, 'FLIGHT COMPUTER', (9, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (185, 192, 200), 1, cv2.LINE_AA)
    cv2.line(panel, (0, 36), (config.STATS_W, 36), (60, 60, 60), 1)

    # ── Status row (full width) ───────────────────────────────────────────────
    sy = 57
    dot_col = (50, 210, 50) if linked else (60, 60, 200)
    cv2.circle(panel, (13, sy - 5), 5, dot_col, -1)
    cv2.putText(panel, 'LINKED' if linked else 'NO LINK', (24, sy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, dot_col, 1, cv2.LINE_AA)
    arm_lbl = 'ARMED' if armed else 'DISARMED'
    arm_col = (40, 230, 40) if armed else (75, 75, 75)
    (aw, _), _ = cv2.getTextSize(arm_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.putText(panel, arm_lbl, (config.STATS_W - aw - 7, sy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, arm_col, 1, cv2.LINE_AA)
    if linked:
        state_lbl = _FLIGHT_STATES.get(flight_state, f"ST{flight_state}")
        mode_lbl  = _FLIGHT_MODES.get(flight_mode_id, f"M{flight_mode_id}")
        # Show "FLYING·STAB" when airborne; plain state label otherwise
        if flight_state in (3, 4):
            center_lbl = f"{state_lbl}\xb7{mode_lbl}"
        else:
            center_lbl = state_lbl
        state_col = _STATE_COLORS.get(flight_state, (150, 150, 150))
        (cw, _), _ = cv2.getTextSize(center_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
        cv2.putText(panel, center_lbl, (config.STATS_W // 2 - cw // 2, sy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, state_col, 1, cv2.LINE_AA)

    CY = sy + 9
    cv2.line(panel, (0, CY), (config.STATS_W, CY), (48, 48, 48), 1)
    cv2.line(panel, (COL, CY), (COL, h - 1), (50, 50, 50), 1)

    # ── Left column: flight instruments ──────────────────────────────────────
    L          = 0
    BATT_NAV_H = 295   # pixels reserved at bottom for power + nav panels
    gauge_area = max(180, h - CY - BATT_NAV_H)
    row_h      = gauge_area // 3

    # Row 1 — Artificial Horizon (full-width, largest gauge)
    ah_r  = max(20, min(row_h // 2 - 18, COL // 2 - 24))
    ah_cx = L + COL // 2
    ah_cy = CY + row_h // 2
    _draw_artificial_horizon(panel, ah_cx, ah_cy,
                             ah_r, roll if linked else 0.0, pitch if linked else 0.0)
    rp_y = ah_cy + ah_r + 14
    cv2.putText(panel, f"R{math.degrees(roll):+.1f}°",
                (L + 8, rp_y), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (140, 140, 140), 1, cv2.LINE_AA)
    ps = f"P{math.degrees(pitch):+.1f}°"
    (pw, _), _ = cv2.getTextSize(ps, cv2.FONT_HERSHEY_SIMPLEX, 0.34, 1)
    cv2.putText(panel, ps, (L + COL - pw - 8, rp_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (140, 140, 140), 1, cv2.LINE_AA)
    sep1 = CY + row_h
    cv2.line(panel, (L, sep1), (L + COL, sep1), (48, 48, 48), 1)

    # Row 2 — Compass + Altitude
    gr1  = max(14, min(row_h // 2 - 24, COL // 4 - 12))
    g1cx = L + COL // 4
    g1cy = sep1 + row_h // 2
    g2cx = L + 3 * COL // 4
    _draw_compass(panel, g1cx, g1cy, gr1, hdg if linked else 0.0)
    _draw_arc_gauge(panel, g2cx, g1cy, gr1,
                    alt if linked else 0.0, 0.0, 80.0, 'ALT', 'm', (80, 200, 80))
    sep2 = CY + 2 * row_h
    cv2.line(panel, (L, sep2), (L + COL, sep2), (48, 48, 48), 1)

    # Row 3 — Airspeed + Vertical Speed
    gr2  = gr1
    g3cx = L + COL // 4
    g3cy = sep2 + row_h // 2
    g4cx = L + 3 * COL // 4
    _draw_arc_gauge(panel, g3cx, g3cy, gr2,
                    spd if linked else 0.0, 0.0, 15.0,
                    'SPEED', 'm/s', (80, 155, 230), warn_frac=0.73)
    _draw_vspeed(panel, g4cx, g3cy, gr2, climb if linked else 0.0)

    # Power & navigation — anchored to bottom of left column
    power_y = h - BATT_NAV_H
    cv2.line(panel, (L, power_y - 4), (L + COL, power_y - 4), (48, 48, 48), 1)
    _draw_battery_panel(panel, L, power_y, COL,
                        voltage_mv if linked else -1,
                        current_ca if linked else -1,
                        batt,
                        mah_con    if linked else -1,
                        time_rem   if linked else 0)
    _draw_distance_home(panel, L, power_y + 155, COL, lat, lon, origin, yaw)

    # ── Right column: systems (motors + payloads) ─────────────────────────────
    R        = COL
    # Motors get everything above payloads and program panel
    payloads_h  = 24 + len(PAYLOAD_NAMES) * 24 + 20
    motors_h    = max(140, h - CY - 4 - payloads_h - OVERLAY_H - 4)
    _draw_motors(panel, R, CY + 4, COL, motors_h,
                 motor_pwm if linked else [0, 0, 0, 0], armed)
    _draw_payloads(panel, R, CY + 4 + motors_h, COL, payload_flags, linked)
    _draw_program_panel(panel, R, h - OVERLAY_H, COL)

    return panel
