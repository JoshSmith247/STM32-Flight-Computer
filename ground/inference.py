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

Architecture is otherwise identical to inference.py (same stream, same optical
flow tracker, same YOLO background thread, same MAVLink stub).
"""

import os
import sys
import math
import json
import socket
import threading
# import queue
import cv2
import numpy as np
# from ultralytics import YOLO
from dotenv import load_dotenv

try:
    from pymavlink import mavutil as _mavutil
    _HAVE_MAVLINK = True
except ImportError:
    _HAVE_MAVLINK = False

load_dotenv()

LAPTOP_IP         = os.environ.get('LAPTOP_IP',        '192.168.4.2')
VIDEO_PORT        = int(os.environ.get('VIDEO_PORT',      '5600'))
GCS_PORT          = int(os.environ.get('GCS_PORT',        '14550'))
PI_IP             = os.environ.get('PI_IP',            '192.168.4.1')
WEED_TARGET_PORT  = int(os.environ.get('WEED_TARGET_PORT', '5700'))
# MODEL_PATH    = os.environ.get('MODEL_PATH',   'yolov8n.pt')
# CONF_THRESH   = float(os.environ.get('CONF_THRESH',  '0.4'))
# IOU_THRESH    = float(os.environ.get('IOU_THRESH',   '0.45'))
# INFER_EVERY_N = int(os.environ.get('INFER_EVERY_N',  '3'))
# INFER_IMGSZ   = int(os.environ.get('INFER_IMGSZ',   '320'))
# DEVICE        = os.environ.get('DEVICE', 'mps')
EXG_THRESH    = float(os.environ.get('EXG_THRESH',  '20.0'))  # 2G-R-B cutoff
EXG_MIN_AREA  = int(os.environ.get('EXG_MIN_AREA',  '250'))   # px² minimum blob
EXG_SAT_MIN   = int(os.environ.get('EXG_SAT_MIN',   '40'))    # HSV saturation floor (0-255); rocks score low here
EXG_MAX_FRAC       = float(os.environ.get('EXG_MAX_FRAC',        '0.04'))  # max blob as fraction of frame area
REMATCH_THRESH     = 0.35    # template correlation floor — no GPS (multiple blobs competing)
REMATCH_THRESH_GPS = 0.12    # template correlation floor — GPS world-gate already fired;
                             #   just confirms something green is at the right location
TEMPLATE_POOL_SIZE = 3       # rolling window of templates kept per weed
MAX_LOST_FRAMES    = 90      # frames (~3 s at 30 fps) before a lost named weed is discarded
CAM_HFOV           = float(os.environ.get('CAM_HFOV',           '62.0'))  # camera horizontal FOV, degrees
WORLD_REMATCH_DIST = float(os.environ.get('WORLD_REMATCH_DIST',  '0.4'))  # metres radius for position-based re-ID

GST_PIPELINE = (
    f"udpsrc port={VIDEO_PORT} "
    f"! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=1"
)
FFMPEG_URL = f"udp://@0.0.0.0:{VIDEO_PORT}?overrun_nonfatal=1&fifo_size=5000000"

LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Sidebar dimensions (drawn at display resolution, appended to the right of the frame)
SIDEBAR_W      = 200
SIDEBAR_HDR_H  = 40   # title bar height
SIDEBAR_ROW_H  = 28   # height per weed entry

# ---------------------------------------------------------------------------
# Weed target sender — UDP to Pi weed_pilot.py
# ---------------------------------------------------------------------------

_target_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_weed_target(wid: int, east_m: float, north_m: float) -> None:
    """Send selected weed's world position to the Pi over UDP."""
    msg = json.dumps({'wid': wid, 'east_m': round(east_m, 3),
                      'north_m': round(north_m, 3)}).encode()
    try:
        _target_sock.sendto(msg, (PI_IP, WEED_TARGET_PORT))
        print(f"Sent W{wid} → Pi: {east_m:.2f}m E, {north_m:.2f}m N", flush=True)
    except OSError as exc:
        print(f"UDP send failed: {exc}", flush=True)


# ---------------------------------------------------------------------------
# MAVLink position receiver
# ---------------------------------------------------------------------------
# Receives GLOBAL_POSITION_INT + ATTITUDE from the drone in a background thread.
# pixel_to_world() uses the latest fix to project pixel centroids to local ENU metres,
# giving each named weed a world-frame position that survives it leaving the frame.

_mav_lock  = threading.Lock()
_mav_state: dict = {'lat': None, 'lon': None, 'alt': 1.0, 'yaw': 0.0, 'origin': None}


def _mav_listener() -> None:
    if not _HAVE_MAVLINK:
        return
    try:
        conn = _mavutil.mavlink_connection(f'udpin:0.0.0.0:{GCS_PORT}')
    except Exception as exc:
        print(f"MAVLink listener failed: {exc}", flush=True)
        return
    print(f"MAVLink listening on UDP port {GCS_PORT}", flush=True)
    while True:
        msg = conn.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'],
                              blocking=True, timeout=1.0)
        if msg is None:
            continue
        with _mav_lock:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = max(0.3, msg.relative_alt / 1000.0)   # mm → m, clamp
                if _mav_state['origin'] is None:
                    _mav_state['origin'] = (lat, lon)
                _mav_state.update(lat=lat, lon=lon, alt=alt)
            elif msg.get_type() == 'ATTITUDE':
                _mav_state['yaw'] = msg.yaw   # radians, NED clockwise from North


def pixel_to_world(px: float, py: float,
                   frame_w: int, frame_h: int) -> tuple[float, float] | None:
    """Project a pixel centroid to local ENU metres (East, North).

    Assumes nadir camera: image-right = drone-right, image-up = drone-forward.
    Returns None until the first MAVLink GLOBAL_POSITION_INT is received.
    """
    with _mav_lock:
        if _mav_state['lat'] is None:
            return None
        lat, lon, alt, yaw, orig = (
            _mav_state['lat'], _mav_state['lon'],
            _mav_state['alt'], _mav_state['yaw'], _mav_state['origin'],
        )

    # Pixel offset from principal point → drone-body metres
    fx    = (frame_w / 2) / math.tan(math.radians(CAM_HFOV / 2))
    scale = alt / fx
    u =  (px - frame_w / 2) * scale   # rightward  (East  when yaw = 0)
    v = -(py - frame_h / 2) * scale   # forward    (North when yaw = 0, y-axis flipped)

    # Rotate body frame into ENU by drone yaw
    east  =  math.cos(yaw) * u + math.sin(yaw) * v
    north = -math.sin(yaw) * u + math.cos(yaw) * v

    # Drone ENU position relative to first fix
    R = 6_371_000.0
    drone_e = math.radians(lon - orig[1]) * R * math.cos(math.radians(orig[0]))
    drone_n = math.radians(lat - orig[0]) * R

    return drone_e + east, drone_n + north


# ---------------------------------------------------------------------------
# ExG pre-filter
# ---------------------------------------------------------------------------

def exg_mask(frame: np.ndarray) -> np.ndarray:
    """Return a binary mask of vegetation pixels.

    Two gates must both pass:
      1. Excess Green: 2G - R - B > EXG_THRESH  (colour arithmetic)
      2. HSV saturation >= EXG_SAT_MIN           (rocks are grey/desaturated; weeds aren't)
    """
    f = frame.astype(np.float32)
    exg = 2.0 * f[:, :, 1] - f[:, :, 2] - f[:, :, 0]
    exg_gate = (exg > EXG_THRESH).astype(np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Hue 25-85 (OpenCV 0-179 scale) covers yellow-green → green → blue-green
    # Saturation floor rejects grey rocks, concrete, and lichen
    sat_gate = cv2.inRange(hsv, (25, EXG_SAT_MIN, 30), (85, 255, 255))

    binary = (exg_gate * sat_gate)   # both conditions must hold
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary


def exg_candidates(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Return (x1, y1, x2, y2) bounding boxes for each green blob within the
    allowed size range, with boxes fully contained inside a larger box removed."""
    frame_area = mask.shape[0] * mask.shape[1]
    max_area   = EXG_MAX_FRAC * frame_area

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if EXG_MIN_AREA <= area <= max_area:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes.append((x, y, x + w, y + h))

    # Drop any box fully contained within another box
    filtered = []
    for i, (ax1, ay1, ax2, ay2) in enumerate(boxes):
        contained = any(
            bx1 <= ax1 and by1 <= ay1 and bx2 >= ax2 and by2 >= ay2
            for j, (bx1, by1, bx2, by2) in enumerate(boxes) if j != i
        )
        if not contained:
            filtered.append((ax1, ay1, ax2, ay2))
    return filtered


# ---------------------------------------------------------------------------
# Optical flow tracker + named-weed re-identification
# ---------------------------------------------------------------------------

class WeedTracker:
    def __init__(self):
        self._tracks: dict = {}            # YOLO tracks (re-enable with YOLO section)
        self._prev_gray = None
        self._exg_boxes: list = []
        self._named: dict[int, dict] = {} # weed_id → {box, template, pts, lost_frames, world_pos}
        self._next_id = 1
        self._pending_click: tuple[int, int] | None = None
        self._frame_size: tuple[int, int] | None = None
        self._selected_wid: int | None = None  # weed shown in red / highlighted in sidebar

    def set_frame_size(self, w: int, h: int) -> None:
        self._frame_size = (w, h)

    # ── YOLO integration (kept for re-enable) ───────────────────────────────

    def update_detections(self, frame: np.ndarray, boxes, names: dict) -> None:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for i, box in enumerate(boxes):
            tid = int(box.id) if box.id is not None else -(i + 1)
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
            roi = gray[y1:y2, x1:x2]
            pts = cv2.goodFeaturesToTrack(roi, maxCorners=30, qualityLevel=0.3, minDistance=5)
            if pts is not None:
                pts[:, 0, 0] += x1
                pts[:, 0, 1] += y1
            self._tracks[tid] = {'pts': pts, 'box': (x1, y1, x2, y2), 'label': names[int(box.cls)]}
        self._prev_gray = gray

    def propagate(self, frame: np.ndarray) -> None:
        """Propagate YOLO tracks via optical flow; keeps _prev_gray current."""
        if self._prev_gray is None or not self._tracks:
            self._prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dead = []
        for tid, track in self._tracks.items():
            pts = track['pts']
            if pts is None or len(pts) < 3:
                dead.append(tid)
                continue
            new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray, gray, pts, None, **LK_PARAMS
            )
            good = status.ravel() == 1
            if good.sum() < 3:
                dead.append(tid)
                continue
            dx = float((new_pts[good] - pts[good]).mean(axis=0)[0, 0])
            dy = float((new_pts[good] - pts[good]).mean(axis=0)[0, 1])
            x1, y1, x2, y2 = track['box']
            track['box'] = (int(x1 + dx), int(y1 + dy), int(x2 + dx), int(y2 + dy))
            track['pts'] = new_pts[good].reshape(-1, 1, 2)
        for tid in dead:
            del self._tracks[tid]
        self._prev_gray = gray

    # ── Named-weed tracking ──────────────────────────────────────────────────

    def _tmatch_one(self, template: np.ndarray, frame: np.ndarray, box: tuple) -> float:
        """Normalized cross-correlation between one template and a candidate blob."""
        if template is None or template.size == 0:
            return 0.0
        x1, y1, x2, y2 = box
        bw, bh = x2 - x1, y2 - y1
        if bw < 4 or bh < 4 or template.shape[1] < 4 or template.shape[0] < 4:
            return 0.0
        tmpl_g = cv2.cvtColor(cv2.resize(template, (bw, bh)), cv2.COLOR_BGR2GRAY)
        cand_g = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
        if cand_g.shape != tmpl_g.shape:
            return 0.0
        result = cv2.matchTemplate(cand_g, tmpl_g, cv2.TM_CCOEFF_NORMED)
        return float(result[0, 0])

    def _tmatch(self, templates: list, frame: np.ndarray, box: tuple) -> float:
        """Best match score across the entire template pool for this weed."""
        return max((self._tmatch_one(t, frame, box) for t in templates), default=0.0)

    @staticmethod
    def _push_template(w: dict, crop: np.ndarray) -> None:
        """Add a new crop to the rolling template pool, evicting the oldest if full."""
        w['templates'].append(crop.copy())
        if len(w['templates']) > TEMPLATE_POOL_SIZE:
            w['templates'].pop(0)

    def _name_box(self, box: tuple, frame: np.ndarray) -> None:
        """Capture template pool, optical flow seed, and world position for a new named weed."""
        x1, y1, x2, y2 = box
        template = frame[y1:y2, x1:x2].copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi  = gray[y1:y2, x1:x2]
        pts  = cv2.goodFeaturesToTrack(roi, maxCorners=20, qualityLevel=0.3, minDistance=4)
        if pts is not None:
            pts[:, 0, 0] += x1
            pts[:, 0, 1] += y1

        world_pos = None
        if self._frame_size is not None:
            world_pos = pixel_to_world((x1 + x2) / 2, (y1 + y2) / 2,
                                       self._frame_size[0], self._frame_size[1])

        wid = self._next_id
        self._next_id += 1
        self._named[wid] = {
            'box': box, 'templates': [template], 'pts': pts,
            'lost_frames': 0, 'world_pos': world_pos,
        }
        loc = (f"{world_pos[0]:.2f}m E, {world_pos[1]:.2f}m N"
               if world_pos else "no GPS fix")
        print(f"Named W{wid}  ({loc})", flush=True)

    def propagate_named(self, frame: np.ndarray, exg_blobs: list) -> None:
        """Track named weeds via optical flow; re-identify lost ones by template matching.

        Called each frame BEFORE propagate() so _prev_gray still holds last frame's gray.
        Pass 1 — optical flow + ExG overlap snaps each named weed to its blob.
        Pass 2 — template matching recovers weeds that went out of view and returned.
        """
        if self._prev_gray is None or not self._named:
            return
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        claimed = set()
        dead    = []

        # Pass 1 — optical flow predicts new position; ExG center check confirms it.
        # The predicted box is NOT written back unless its center lands inside a blob,
        # so a drifting flow can't silently claim an unrelated green patch.
        for wid, w in self._named.items():
            # Predict where the box moved this frame
            prop_box = w['box']
            pts = w['pts']
            if pts is not None and len(pts) >= 3:
                new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self._prev_gray, gray, pts, None, **LK_PARAMS
                )
                good = status.ravel() == 1
                if good.sum() >= 3:
                    dx = float((new_pts[good] - pts[good]).mean(axis=0)[0, 0])
                    dy = float((new_pts[good] - pts[good]).mean(axis=0)[0, 1])
                    x1, y1, x2, y2 = w['box']
                    prop_box = (int(x1+dx), int(y1+dy), int(x2+dx), int(y2+dy))

            # Only claim a blob if the predicted center actually lands inside it
            pbx1, pby1, pbx2, pby2 = prop_box
            pcx, pcy = (pbx1 + pbx2) / 2, (pby1 + pby2) / 2
            best_blob = None
            for i, (ex1, ey1, ex2, ey2) in enumerate(exg_blobs):
                if i in claimed:
                    continue
                if ex1 <= pcx <= ex2 and ey1 <= pcy <= ey2:
                    best_blob = i
                    break

            if best_blob is not None:
                ex1, ey1, ex2, ey2 = exg_blobs[best_blob]
                w['box'] = (ex1, ey1, ex2, ey2)
                roi = gray[ey1:ey2, ex1:ex2]
                pts2 = cv2.goodFeaturesToTrack(roi, maxCorners=20, qualityLevel=0.3, minDistance=4)
                if pts2 is not None:
                    pts2[:, 0, 0] += ex1
                    pts2[:, 0, 1] += ey1
                w['pts'] = pts2
                self._push_template(w, frame[ey1:ey2, ex1:ex2])
                if self._frame_size is not None:
                    wp = pixel_to_world((ex1 + ex2) / 2, (ey1 + ey2) / 2,
                                        self._frame_size[0], self._frame_size[1])
                    if wp is not None:
                        w['world_pos'] = wp
                w['lost_frames'] = 0
                claimed.add(best_blob)
            else:
                # No confirmed blob — freeze box at last known position and clear flow
                # points so drift doesn't accumulate while the weed is out of view.
                w['pts'] = None
                w['lost_frames'] += 1
                # Only expire weeds that never got a GPS fix — those with a world
                # position are remembered indefinitely and re-identified on re-entry.
                # if w['lost_frames'] > MAX_LOST_FRAMES and w['world_pos'] is None:
                #     dead.append(wid)

        # Pass 2 — re-identification for lost named weeds.
        # World position gates candidates to blobs within WORLD_REMATCH_DIST metres;
        # template matching then picks the best among those (graceful fallback when
        # no GPS fix is available: all blobs are considered).
        for wid, w in self._named.items():
            if wid in dead or w['lost_frames'] == 0:
                continue
            candidates: list[tuple[float, int]] = []
            for i, box in enumerate(exg_blobs):
                if i in claimed:
                    continue
                world_confirmed = False
                if w['world_pos'] is not None and self._frame_size is not None:
                    bcx = (box[0] + box[2]) / 2
                    bcy = (box[1] + box[3]) / 2
                    wp  = pixel_to_world(bcx, bcy,
                                         self._frame_size[0], self._frame_size[1])
                    if wp is not None:
                        dist = math.hypot(wp[0] - w['world_pos'][0],
                                          wp[1] - w['world_pos'][1])
                        if dist > WORLD_REMATCH_DIST:
                            continue   # too far in the world
                        world_confirmed = True
                # GPS-confirmed candidates need only a loose template sanity check;
                # without GPS, template matching is the only discriminator so we
                # require a stricter score to avoid cross-ID between similar weeds.
                thresh = REMATCH_THRESH_GPS if world_confirmed else REMATCH_THRESH
                score  = self._tmatch(w['templates'], frame, box)
                if score > thresh:
                    candidates.append((score, i))
            if not candidates:
                continue
            best_score, best_blob = max(candidates)
            ex1, ey1, ex2, ey2 = exg_blobs[best_blob]
            w['box'] = (ex1, ey1, ex2, ey2)
            roi = gray[ey1:ey2, ex1:ex2]
            pts2 = cv2.goodFeaturesToTrack(roi, maxCorners=20, qualityLevel=0.3, minDistance=4)
            if pts2 is not None:
                pts2[:, 0, 0] += ex1
                pts2[:, 0, 1] += ey1
            w['pts'] = pts2
            self._push_template(w, frame[ey1:ey2, ex1:ex2])
            if self._frame_size is not None:
                wp = pixel_to_world((ex1 + ex2) / 2, (ey1 + ey2) / 2,
                                     self._frame_size[0], self._frame_size[1])
                if wp is not None:
                    w['world_pos'] = wp
            w['lost_frames'] = 0
            claimed.add(best_blob)
            print(f"Re-identified W{wid} (score={best_score:.2f})", flush=True)

        for wid in dead:
            print(f"Expired W{wid} — no GPS fix and not seen for {MAX_LOST_FRAMES} frames", flush=True)
            del self._named[wid]

    # ── Click handling ───────────────────────────────────────────────────────

    def queue_click(self, x: int, y: int) -> None:
        """Store click coordinates; resolved with the live frame in process_click()."""
        self._pending_click = (x, y)

    def process_click(self, frame: np.ndarray) -> None:
        """Name an ExG blob on click, or remove a named weed if clicked again."""
        if self._pending_click is None:
            return
        x, y = self._pending_click
        self._pending_click = None

        # Click a named weed → remove it
        for wid in list(self._named):
            x1, y1, x2, y2 = self._named[wid]['box']
            if x1 <= x <= x2 and y1 <= y <= y2:
                print(f"Removed W{wid}", flush=True)
                del self._named[wid]
                return

        # Click an unnamed ExG blob → assign next ID
        named_boxes = {w['box'] for w in self._named.values()}
        for box in self._exg_boxes:
            if box in named_boxes:
                continue
            x1, y1, x2, y2 = box
            if x1 <= x <= x2 and y1 <= y <= y2:
                self._name_box(box, frame)
                return

    # ── ExG list update ──────────────────────────────────────────────────────

    def update_exg(self, boxes: list) -> None:
        self._exg_boxes = boxes

    # ── Rendering ────────────────────────────────────────────────────────────

    def draw(self, frame: np.ndarray, exg_boxes: list) -> np.ndarray:
        out = frame.copy()
        named_boxes = {w['box'] for w in self._named.values()}

        # Named weeds — red when selected, teal when active, grey when lost
        for wid, w in self._named.items():
            x1, y1, x2, y2 = w['box']
            active   = w['lost_frames'] == 0
            selected = wid == self._selected_wid
            if selected:
                color, label, thick = (0, 0, 220), f"W{wid}", 3   # red (BGR)
            elif active:
                color, label, thick = (0, 230, 160), f"W{wid}", 2  # teal
            else:
                color, label, thick = (90, 90, 90), f"W{wid}?", 2  # grey
            cv2.rectangle(out, (x1, y1), (x2, y2), color, thick)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
            cv2.putText(out, label, (x1 + 3, y1 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # Unnamed ExG blobs — orange
        for box in exg_boxes:
            if box in named_boxes:
                continue
            x1, y1, x2, y2 = box
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 140, 255), 2)

        # YOLO tracks (currently unused, kept for re-enable)
        for tid, track in self._tracks.items():
            x1, y1, x2, y2 = track['box']
            cv2.rectangle(out, (x1, y1), (x2, y2), (200, 0, 200), 2)
            cv2.putText(out, f"{track['label']} #{tid}",
                        (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1, cv2.LINE_AA)
        return out

    def sidebar_click(self, y: int) -> None:
        """Toggle selection of the weed at sidebar row y."""
        if y < SIDEBAR_HDR_H:
            return
        idx  = (y - SIDEBAR_HDR_H) // SIDEBAR_ROW_H
        wids = sorted(self._named.keys())
        if 0 <= idx < len(wids):
            wid = wids[idx]
            self._selected_wid = None if self._selected_wid == wid else wid

    def draw_sidebar(self, h: int) -> np.ndarray:
        """Return a SIDEBAR_W × h panel listing all named weeds."""
        panel = np.full((h, SIDEBAR_W, 3), (35, 35, 35), dtype=np.uint8)

        # Title bar
        cv2.rectangle(panel, (0, 0), (SIDEBAR_W, SIDEBAR_HDR_H - 1), (55, 55, 55), -1)
        cv2.putText(panel, "Named Weeds", (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.line(panel, (0, SIDEBAR_HDR_H - 1), (SIDEBAR_W, SIDEBAR_HDR_H - 1),
                 (70, 70, 70), 1)

        mid = SIDEBAR_ROW_H // 2
        for i, wid in enumerate(sorted(self._named.keys())):
            w        = self._named[wid]
            y0       = SIDEBAR_HDR_H + i * SIDEBAR_ROW_H
            active   = w['lost_frames'] == 0
            selected = wid == self._selected_wid

            # Row highlight for selected weed
            if selected:
                cv2.rectangle(panel, (2, y0 + 1), (SIDEBAR_W - 2, y0 + SIDEBAR_ROW_H - 1),
                              (60, 25, 25), -1)

            # Status dot
            dot_color = (50, 200, 80) if active else (70, 70, 70)
            cv2.circle(panel, (12, y0 + mid), 4, dot_color, -1)

            # Weed label
            label = f"W{wid}" if active else f"W{wid}?"
            if selected:
                text_color = (80, 80, 220)    # red-ish (BGR) for selected
            elif active:
                text_color = (180, 220, 180)  # light green for active
            else:
                text_color = (100, 100, 100)  # grey for lost
            cv2.putText(panel, label, (22, y0 + mid + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, text_color, 1, cv2.LINE_AA)

            # GPS indicator
            if w['world_pos']:
                cv2.putText(panel, "GPS", (SIDEBAR_W - 38, y0 + mid + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 150, 80), 1, cv2.LINE_AA)

        return panel

    def centroids(self) -> list[tuple[float, float]]:
        result = []
        for track in self._tracks.values():
            x1, y1, x2, y2 = track['box']
            result.append(((x1 + x2) / 2, (y1 + y2) / 2))
        return result


# ---------------------------------------------------------------------------
# Background YOLO inference thread (commented out — ExG-only mode)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# MAVLink targeting stub (commented out — ExG-only mode)
# ---------------------------------------------------------------------------

# def send_target(sock: socket.socket, x_norm: float, y_norm: float) -> None:
#     print(f"Target offset: x={x_norm:+.2f} y={y_norm:+.2f}", flush=True)
#     # TODO: sock.sendto(mavlink_frame, (PI_IP, GCS_PORT))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("GStreamer unavailable, trying ffmpeg backend...", flush=True)
        cap = cv2.VideoCapture(FFMPEG_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("Failed to open video stream — is demo_cam.py or camera_stream.py running?",
              flush=True)
        sys.exit(1)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Stream: {frame_w}×{frame_h}", flush=True)
    print(f"ExG threshold: {EXG_THRESH}  min blob: {EXG_MIN_AREA}px²", flush=True)
    print("Keys: 'e' ExG overlay  'q' quit  |  click blob to name (W1,W2…), click name to remove", flush=True)

    if _HAVE_MAVLINK:
        threading.Thread(target=_mav_listener, daemon=True).start()
    else:
        print("pymavlink not installed — world-frame re-ID disabled (pip install pymavlink)", flush=True)

    DISPLAY_LONG = 720
    disp_scale = DISPLAY_LONG / max(frame_w, frame_h)
    disp_w = int(frame_w * disp_scale)
    disp_h = int(frame_h * disp_scale)

    tracker  = WeedTracker()
    tracker.set_frame_size(frame_w, frame_h)
    show_exg      = False
    prev_selected: int | None = None

    WIN = "Weed Detection — ExG"
    cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if x >= disp_w:
            # Click in sidebar — toggle weed selection
            tracker.sidebar_click(y)
        else:
            # Click in camera feed — name / remove weed
            tracker.queue_click(int(x / disp_scale), int(y / disp_scale))

    cv2.setMouseCallback(WIN, on_mouse)

    # worker = threading.Thread(target=_inference_worker, daemon=True)
    # worker.start()

    frame_count  = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame_count += 1

        # ExG pre-filter — always runs
        mask  = exg_mask(frame)
        blobs = exg_candidates(mask)

        # YOLO gating and inference commented out — ExG-only mode
        # if blobs and frame_count % INFER_EVERY_N == 0:
        #     try:
        #         _infer_queue.put_nowait(frame)
        #     except queue.Full:
        #         pass
        # with _det_lock:
        #     det = _latest_det
        #     globals()['_latest_det'] = None
        # if det is not None:
        #     det_frame, det_boxes, names = det
        #     tracker.update_detections(det_frame, det_boxes, names)

        tracker.process_click(frame)           # name/unname blobs on click
        tracker.propagate_named(frame, blobs)  # optical flow + re-ID (uses prev gray)
        tracker.propagate(frame)               # updates _prev_gray for next frame
        tracker.update_exg(blobs)
        display = tracker.draw(frame, blobs)

        # ExG overlay (toggled with 'e')
        if show_exg:
            tint = np.zeros_like(display)
            tint[:, :, 1] = mask   # green channel only
            display = cv2.addWeighted(display, 1.0, tint, 0.4, 0)

        display = cv2.resize(display, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)

        # HUD
        n_active = sum(1 for w in tracker._named.values() if w['lost_frames'] == 0)
        n_lost   = len(tracker._named) - n_active
        hud = f"ExG blobs: {len(blobs)}"
        if n_active:
            hud += f"  |  named: {n_active}"
        if n_lost:
            hud += f"  |  searching: {n_lost}"
        cv2.putText(display, hud, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        # Sidebar — appended to the right; mouse callback uses disp_w as the boundary
        sidebar  = tracker.draw_sidebar(disp_h)
        combined = np.hstack([display, sidebar])

        # Send weed target to Pi when the operator selects (or changes) a weed
        sel = tracker._selected_wid
        if sel != prev_selected:
            prev_selected = sel
            if sel is not None:
                wp = tracker._named.get(sel, {}).get('world_pos')
                if wp is not None:
                    send_weed_target(sel, wp[0], wp[1])
                else:
                    print(f"W{sel} selected — no GPS fix yet, target not sent", flush=True)

        cv2.imshow(WIN, combined)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('e'):
            show_exg = not show_exg
            print(f"ExG overlay {'on' if show_exg else 'off'}", flush=True)

    # _infer_queue.put(None)
    cap.release()
    cv2.destroyAllWindows()
    _target_sock.close()
    print(f"\nDone. {frame_count} frames processed.", flush=True)


if __name__ == '__main__':
    main()
