#!/usr/bin/env python3
"""
Top-down weed map builder.

In live mode, subscribes to MAVLink GLOBAL_POSITION_INT + ATTITUDE on GCS_PORT
and uses real-world GPS coordinates to place each frame on the canvas — no ORB
feature matching needed, no accumulated drift.  Falls back to ORB visual odometry
when GPS is unavailable (offline mode or pymavlink not installed).

ExG detections are accumulated as a binary mask; each weed blob is assigned a
lat/lon centroid and exported as GeoJSON alongside the PNG.

Usage:
    python weed_map.py                     # live UDP stream + GPS georeferencing
    python weed_map.py path/to/video.mp4  # offline — ORB odometry, no GeoJSON

Keys:
    s  — save current map + GeoJSON to weed_map_<timestamp>.{png,geojson}
    q  — save and quit

Outputs:
    weed_map.png      — overhead RGB mosaic with ExG weed overlay, scale bar, north arrow
    weed_map.geojson  — GeoJSON FeatureCollection; one Point per weed centroid with area_m2

Env vars (in addition to VIDEO_PORT / GCS_PORT from .env):
    CAM_HFOV    horizontal field-of-view of the lens in degrees (default 62.0)
    MAP_SCALE   canvas pixels per metre (default 20.0; raise for more detail)
"""

import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
from dotenv import load_dotenv

from exg import exg_mask

load_dotenv()

VIDEO_PORT = int(os.environ.get('VIDEO_PORT', '5600'))
GCS_PORT   = int(os.environ.get('GCS_PORT',   '14550'))
CAM_HFOV   = float(os.environ.get('CAM_HFOV',   '62.0'))   # degrees
MAP_SCALE  = float(os.environ.get('MAP_SCALE',   '20.0'))   # canvas px per metre

GST_PIPELINE = (
    f"udpsrc port={VIDEO_PORT} "
    f"! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=1"
)
FFMPEG_URL = f"udp://@0.0.0.0:{VIDEO_PORT}?overrun_nonfatal=1&fifo_size=5000000"

# Canvas covers ±(CANVAS_W/2 / MAP_SCALE) metres from origin in each axis.
# At 20 px/m, 4000×4000 px covers a 200×200 m patch — enough for a large garden.
CANVAS_W = 4000
CANVAS_H = 4000
WORK_W   = 640   # frames are scaled to this width before processing


# ---------------------------------------------------------------------------
# GPS receiver — live mode only
# ---------------------------------------------------------------------------

class GpsReceiver:
    """Receives MAVLink GLOBAL_POSITION_INT + ATTITUDE in a background thread."""

    def __init__(self, port: int) -> None:
        self._lock   = threading.Lock()
        self._lat:    float | None = None
        self._lon:    float | None = None
        self._alt:    float        = 1.0
        self._yaw:    float        = 0.0
        self._origin: tuple | None = None
        threading.Thread(target=self._listen, args=(port,), daemon=True).start()

    def _listen(self, port: int) -> None:
        try:
            from pymavlink import mavutil
        except ImportError:
            print("pymavlink not installed — GPS integration disabled (pip install pymavlink)",
                  flush=True)
            return
        try:
            conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')
        except Exception as exc:
            print(f"GPS receiver failed to open port {port}: {exc}", flush=True)
            return
        print(f"GPS receiver listening on MAVLink UDP:{port}", flush=True)
        while True:
            msg = conn.recv_match(
                type=['GLOBAL_POSITION_INT', 'ATTITUDE'],
                blocking=True, timeout=1.0)
            if msg is None:
                continue
            with self._lock:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    self._lat = msg.lat / 1e7
                    self._lon = msg.lon / 1e7
                    self._alt = max(0.3, msg.relative_alt / 1000.0)
                    if self._origin is None:
                        self._origin = (self._lat, self._lon)
                elif msg.get_type() == 'ATTITUDE':
                    self._yaw = msg.yaw

    def get(self) -> tuple | None:
        """Return (lat, lon, alt_m, yaw_rad, origin_tuple) or None if no fix yet."""
        with self._lock:
            if self._lat is None or self._origin is None:
                return None
            return (self._lat, self._lon, self._alt, self._yaw, self._origin)


# ---------------------------------------------------------------------------
# Mosaic builder
# ---------------------------------------------------------------------------

class MosaicBuilder:
    """
    Accumulates camera frames into an overhead mosaic.

    GPS mode (live):  each frame is placed via a similarity transform derived
      from the drone's GPS position, altitude, and heading.  Zero drift.

    ORB mode (offline / no GPS):  each frame is registered to the previous
      one via ORB keypoint matching + RANSAC homography.

    Axis convention (landscape, unrotated frame):
      image +X (right) = body +Y (right side of drone)
      image +Y (down)  = body -X (backward; image UP = body forward)
    This matches the 90°-CW-rotated convention used in pixel_to_world() and
    corresponds to the camera's top edge pointing toward the drone's nose.
    """

    def __init__(self, px_per_metre: float = MAP_SCALE,
                 cam_hfov_deg: float = CAM_HFOV) -> None:
        self._px_m  = px_per_metre
        self._hfov  = math.radians(cam_hfov_deg)

        self._canvas_cx = CANVAS_W // 2
        self._canvas_cy = CANVAS_H // 2
        self._origin: tuple | None = None   # (lat_deg, lon_deg) at canvas centre

        self._canvas_rgb = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
        self._canvas_exg = np.zeros((CANVAS_H, CANVAS_W),    dtype=np.uint8)
        self._canvas_hit = np.zeros((CANVAS_H, CANVAS_W),    dtype=np.uint8)

        # ORB fallback state
        self._H         = np.eye(3, dtype=np.float64)
        self._H_init    = False
        self._prev_kp   = None
        self._prev_des  = None
        self._orb     = cv2.ORB_create(nfeatures=1000)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self._frame_count = 0
        self._gps_frames  = 0

    # ── GPS homography ────────────────────────────────────────────────────────

    def _gps_H(self, lat: float, lon: float, alt: float, yaw: float,
               frame_w: int, frame_h: int) -> np.ndarray:
        """Similarity transform: frame pixels → canvas pixels.

        Derivation:
          pixel offset from frame centre → body NED (using HFOV + altitude) →
          ENU displacement → canvas pixel position.

          body_forward = -(py - h/2) * s       (image UP = forward)
          body_right   =  (px - w/2) * s       (image RIGHT = right)
          east   = body_fwd * sin(ψ) + body_rgt * cos(ψ)
          north  = body_fwd * cos(ψ) - body_rgt * sin(ψ)
          cx = canvas_cx + (drone_e + east ) * px_m
          cy = canvas_cy - (drone_n + north) * px_m   (canvas y increases downward)

        Expands to the similarity matrix below.
        """
        if self._origin is None:
            self._origin = (lat, lon)

        R_earth = 6_371_000.0
        lat0, lon0 = self._origin
        drone_e = math.radians(lon - lon0) * R_earth * math.cos(math.radians(lat0))
        drone_n = math.radians(lat - lat0) * R_earth

        # metres per frame pixel (HFOV spans frame_w pixels)
        scale_mpp = alt * math.tan(self._hfov / 2) / (frame_w / 2)
        k = scale_mpp * self._px_m   # canvas pixels per frame pixel

        cy_v, sy_v = math.cos(yaw), math.sin(yaw)
        tx = (self._canvas_cx + drone_e * self._px_m
              - k * (frame_w / 2 * cy_v - frame_h / 2 * sy_v))
        ty = (self._canvas_cy - drone_n * self._px_m
              - k * (frame_h / 2 * cy_v + frame_w / 2 * sy_v))
        return np.array([
            [ k * cy_v, -k * sy_v, tx],
            [ k * sy_v,  k * cy_v, ty],
            [0,          0,         1],
        ], dtype=np.float64)

    # ── ORB fallback ──────────────────────────────────────────────────────────

    def _orb_update_H(self, gray: np.ndarray) -> None:
        kp, des = self._orb.detectAndCompute(gray, None)
        if (self._prev_des is not None and des is not None
                and len(self._prev_des) >= 6 and len(des) >= 6):
            matches = self._matcher.match(self._prev_des, des)
            matches = sorted(matches, key=lambda m: m.distance)[:80]
            if len(matches) >= 4:
                src_pts = np.float32(
                    [self._prev_kp[m.queryIdx].pt for m in matches]
                ).reshape(-1, 1, 2)
                dst_pts = np.float32(
                    [kp[m.trainIdx].pt for m in matches]
                ).reshape(-1, 1, 2)
                H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                if H is not None:
                    try:
                        self._H = self._H @ np.linalg.inv(H)
                    except np.linalg.LinAlgError:
                        pass  # degenerate homography — keep previous transform
        self._prev_kp  = kp
        self._prev_des = des

    # ── Public API ────────────────────────────────────────────────────────────

    def add_frame(self, frame: np.ndarray,
                  gps: tuple | None = None) -> None:
        """Add one frame to the mosaic.

        gps: (lat, lon, alt_m, yaw_rad, origin) from GpsReceiver.get(), or None.
        """
        h_src, w_src = frame.shape[:2]
        if w_src != WORK_W:
            scale  = WORK_W / w_src
            frame  = cv2.resize(frame, (WORK_W, int(h_src * scale)),
                                interpolation=cv2.INTER_AREA)
        h, w = frame.shape[:2]

        if gps is not None:
            lat, lon, alt, yaw, origin = gps
            if self._origin is None:
                self._origin = origin
            self._H = self._gps_H(lat, lon, alt, yaw, w, h)
            self._gps_frames += 1
        else:
            if not self._H_init:
                self._H[0, 2] = CANVAS_W // 2 - w // 2
                self._H[1, 2] = CANVAS_H // 2 - h // 2
                self._H_init  = True
            self._orb_update_H(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

        self._frame_count += 1

        kw = dict(flags=cv2.INTER_LINEAR,  borderMode=cv2.BORDER_CONSTANT)
        kn = dict(flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        warped_rgb = cv2.warpPerspective(frame, self._H, (CANVAS_W, CANVAS_H), **kw)
        warped_hit = cv2.warpPerspective(
            np.full((h, w), 255, dtype=np.uint8), self._H, (CANVAS_W, CANVAS_H), **kn)
        warped_exg = cv2.warpPerspective(
            exg_mask(frame), self._H, (CANVAS_W, CANVAS_H), **kn)

        hit = warped_hit > 0
        self._canvas_rgb[hit] = warped_rgb[hit]
        self._canvas_exg = np.maximum(self._canvas_exg, warped_exg)
        self._canvas_hit[hit] = 255

    def render(self) -> np.ndarray | None:
        """Return a cropped composite with ExG overlay, scale bar, and north arrow."""
        rows = np.any(self._canvas_hit, axis=1)
        cols = np.any(self._canvas_hit, axis=0)
        if not rows.any():
            return None

        r0 = int(np.where(rows)[0][0]);  r1 = int(np.where(rows)[0][-1])
        c0 = int(np.where(cols)[0][0]);  c1 = int(np.where(cols)[0][-1])
        pad = 40
        r0 = max(0, r0 - pad);  r1 = min(CANVAS_H, r1 + pad)
        c0 = max(0, c0 - pad);  c1 = min(CANVAS_W, c1 + pad)

        rgb = self._canvas_rgb[r0:r1, c0:c1].copy()
        exg = self._canvas_exg[r0:r1, c0:c1]

        tint = np.zeros_like(rgb)
        tint[exg > 0] = (0, 180, 0)
        out = cv2.addWeighted(rgb, 1.0, tint, 0.45, 0)

        contours, _ = cv2.findContours(exg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 60), 1)

        oh, ow = out.shape[:2]

        # North arrow (top-right) — only meaningful in GPS mode
        if self._gps_frames > 0:
            nax, nay = ow - 22, 32
            cv2.arrowedLine(out, (nax, nay + 16), (nax, nay - 16),
                            (60, 60, 210), 2, tipLength=0.35)
            cv2.putText(out, 'N', (nax - 5, nay - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 60, 210), 1, cv2.LINE_AA)

        # Scale bar (bottom-left): pick smallest round-number length ≥ 60 px
        bar_m = 1.0
        for m in (0.25, 0.5, 1, 2, 5, 10, 20, 50, 100):
            if m * self._px_m >= 60:
                bar_m = m
                break
        bar_px = int(bar_m * self._px_m)
        bx, by = 12, oh - 14
        cv2.line(out, (bx, by), (bx + bar_px, by), (210, 210, 210), 2)
        cv2.line(out, (bx, by - 4), (bx, by + 4), (210, 210, 210), 2)
        cv2.line(out, (bx + bar_px, by - 4), (bx + bar_px, by + 4), (210, 210, 210), 2)
        lbl = f"{bar_m:.3g} m"
        cv2.putText(out, lbl, (bx, by - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (210, 210, 210), 1, cv2.LINE_AA)

        # Mode indicator (top-left)
        mode = (f"GPS ({self._gps_frames} frames, {self._px_m:.0f} px/m)"
                if self._gps_frames else "ORB odometry")
        cv2.putText(out, mode, (8, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (140, 140, 140), 1, cv2.LINE_AA)

        return out

    def weed_centroids_latlon(self) -> list[dict]:
        """Return [{lat, lon, area_m2}, …] for every ExG blob in the full canvas.

        Only available when GPS origin is known (live mode).  Returns empty list
        otherwise — GeoJSON export is skipped in that case.
        """
        if self._origin is None:
            return []
        lat0, lon0 = self._origin
        results = []
        cnts, _ = cv2.findContours(
            self._canvas_exg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(cnts):
            area_px = cv2.contourArea(cnt)
            if area_px < 4:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            # Canvas pixel → ENU metres relative to origin
            east_m  = (cx - self._canvas_cx) / self._px_m
            north_m = -(cy - self._canvas_cy) / self._px_m   # canvas y increases south
            # ENU → lat/lon (flat-earth approximation, valid for patches < 10 km)
            lat = lat0 + north_m / 111_320.0
            lon = lon0 + east_m  / (111_320.0 * math.cos(math.radians(lat0)))
            area_m2 = area_px / (self._px_m ** 2)
            results.append({'lat': round(lat, 8), 'lon': round(lon, 8),
                             'area_m2': round(area_m2, 4)})
        return results

    def save(self, path: str | None = None) -> str:
        img = self.render()
        if img is None:
            print("No frames processed yet — nothing to save.", flush=True)
            return ''
        if path is None:
            path = f"weed_map_{int(time.time())}.png"

        cv2.imwrite(path, img)
        print(f"Saved → {path}  ({img.shape[1]}×{img.shape[0]} px)", flush=True)

        centroids = self.weed_centroids_latlon()
        if centroids:
            geo_path = path.replace('.png', '.geojson')
            features = [
                {
                    'type': 'Feature',
                    'geometry': {'type': 'Point', 'coordinates': [w['lon'], w['lat']]},
                    'properties': {'id': i + 1, 'area_m2': w['area_m2']},
                }
                for i, w in enumerate(centroids)
            ]
            with open(geo_path, 'w') as f:
                json.dump({'type': 'FeatureCollection', 'features': features}, f, indent=2)
            print(f"GeoJSON → {geo_path}  ({len(centroids)} weeds)", flush=True)
        elif self._origin is None:
            print("No GPS origin — GeoJSON skipped (run live for georeferenced output)",
                  flush=True)
        return path

    @property
    def frame_count(self) -> int:
        return self._frame_count


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    offline = len(sys.argv) > 1

    if offline:
        cap = cv2.VideoCapture(sys.argv[1])
        gps_rx = None
    else:
        LIVE_RETRY_S = int(os.environ.get('LIVE_RETRY_S', '30'))
        deadline = time.time() + LIVE_RETRY_S
        cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
        while not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(FFMPEG_URL, cv2.CAP_FFMPEG)
            if cap.isOpened() or time.time() >= deadline:
                break
            print(f"Waiting for stream on UDP:{VIDEO_PORT} ...", flush=True)
            cap.release()
            time.sleep(2)
            cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
        gps_rx = GpsReceiver(GCS_PORT)

    if not cap.isOpened():
        print("Failed to open video source.", flush=True)
        sys.exit(1)

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if offline else 0
    print(f"{'Offline' if offline else 'Live'} source open"
          + (f" — {total} frames" if total > 0 else ""), flush=True)
    print(f"Scale: {MAP_SCALE} px/m  HFOV: {CAM_HFOV}°  "
          f"Coverage: ±{CANVAS_W / 2 / MAP_SCALE:.0f}m from origin", flush=True)

    builder      = MosaicBuilder()
    WIN          = "Weed Map  [s=save  q=quit]"
    raw_count    = 0
    PROCESS_EVERY = 2
    DISPLAY_EVERY = 20

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 900, 900)

    while True:
        ok, frame = cap.read()
        if not ok:
            if offline:
                break
            continue

        raw_count += 1
        if raw_count % PROCESS_EVERY != 0:
            continue

        gps_fix = gps_rx.get() if gps_rx is not None else None
        builder.add_frame(frame, gps=gps_fix)

        if builder.frame_count % DISPLAY_EVERY == 0:
            img = builder.render()
            if img is not None:
                h, w   = img.shape[:2]
                scale  = min(880 / w, 880 / h)
                disp   = cv2.resize(img, (int(w * scale), int(h * scale)),
                                    interpolation=cv2.INTER_AREA)
                if offline and total > 0:
                    pct = int(100 * raw_count / total)
                    cv2.putText(disp, f"{pct}%  ({builder.frame_count} frames)",
                                (8, disp.shape[0] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (180, 180, 180), 1, cv2.LINE_AA)
                cv2.imshow(WIN, disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            builder.save()

    print(f"\nProcessed {builder.frame_count} frames. Saving final map...", flush=True)
    builder.save('weed_map.png')
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
