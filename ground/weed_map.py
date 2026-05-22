#!/usr/bin/env python3
"""
Top-down weed map builder.

Stitches camera frames into an overhead mosaic using ORB feature matching,
then overlays ExG detections as a green heat-map showing the surface area
covered by each weed.

Usage:
    python weed_map.py                     # live UDP stream (same ports as inference2)
    python weed_map.py path/to/video.mp4  # offline — processes whole file then saves

Keys:
    s  — save current map to weed_map_<timestamp>.png
    q  — save and quit

Output PNG shows the stitched ground texture with weed coverage highlighted
in green and outlined in bright green for clarity.
"""

import os
import sys
import time
import cv2
import numpy as np
from dotenv import load_dotenv

load_dotenv()

VIDEO_PORT   = int(os.environ.get('VIDEO_PORT',  '5600'))
EXG_THRESH   = float(os.environ.get('EXG_THRESH',  '20.0'))
EXG_MIN_AREA = int(os.environ.get('EXG_MIN_AREA',  '250'))
EXG_SAT_MIN  = int(os.environ.get('EXG_SAT_MIN',   '40'))
EXG_MAX_FRAC = float(os.environ.get('EXG_MAX_FRAC', '0.04'))

GST_PIPELINE = (
    f"udpsrc port={VIDEO_PORT} "
    f"! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=1"
)
FFMPEG_URL = f"udp://@0.0.0.0:{VIDEO_PORT}?overrun_nonfatal=1&fifo_size=5000000"

# Canvas large enough for the drone to cover a typical garden patch.
# Frames are scaled to WORK_W before processing to keep warpPerspective fast.
CANVAS_W = 4000
CANVAS_H = 4000
WORK_W   = 640   # process at this width regardless of source resolution


# ---------------------------------------------------------------------------
# ExG detection  (same logic as inference2.py)
# ---------------------------------------------------------------------------

def exg_mask(frame: np.ndarray) -> np.ndarray:
    f = frame.astype(np.float32)
    exg = 2.0 * f[:, :, 1] - f[:, :, 2] - f[:, :, 0]
    exg_gate = (exg > EXG_THRESH).astype(np.uint8)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    sat_gate = cv2.inRange(hsv, (25, EXG_SAT_MIN, 30), (85, 255, 255))
    binary = exg_gate * sat_gate
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary


# ---------------------------------------------------------------------------
# Mosaic builder
# ---------------------------------------------------------------------------

class MosaicBuilder:
    """
    Accumulates camera frames into a single overhead mosaic.

    Each frame is registered against the previous one via ORB keypoint matching
    and RANSAC homography estimation.  The accumulated transform H maps the
    current frame's coordinate space to the fixed canvas coordinate space,
    so each new frame is warped into its correct place on the canvas.

    ExG detections are warped with the same transform and composited as a
    binary mask — any pixel that was ever green ends up marked on the map.
    """

    def __init__(self) -> None:
        self._canvas_rgb = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
        self._canvas_exg = np.zeros((CANVAS_H, CANVAS_W),    dtype=np.uint8)
        self._canvas_hit = np.zeros((CANVAS_H, CANVAS_W),    dtype=np.uint8)

        # Seed H so the first frame lands in the canvas centre
        self._H = np.eye(3, dtype=np.float64)
        self._H[0, 2] = CANVAS_W // 2 - WORK_W // 2
        self._H[1, 2] = CANVAS_H // 2 - WORK_W // 2   # square crop assumption; adjusted below

        self._prev_gray = None
        self._prev_kp   = None
        self._prev_des  = None

        self._orb     = cv2.ORB_create(nfeatures=1000)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self._frame_count = 0
        self._first_h = 0   # actual height of first frame, for canvas centering

    def _scale(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        if w == WORK_W:
            return frame
        scale = WORK_W / w
        return cv2.resize(frame, (WORK_W, int(h * scale)), interpolation=cv2.INTER_AREA)

    def add_frame(self, frame: np.ndarray) -> None:
        frame = self._scale(frame)
        h, w  = frame.shape[:2]

        # Fix canvas centering once we know the actual frame height
        if self._frame_count == 0:
            self._first_h = h
            self._H[1, 2] = CANVAS_H // 2 - h // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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
                # H maps prev-frame coords → current-frame coords.
                # We need current-frame → canvas, so compose with H_inv.
                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                if H is not None:
                    self._H = self._H @ np.linalg.inv(H)

        self._prev_gray = gray
        self._prev_kp   = kp
        self._prev_des  = des
        self._frame_count += 1

        # Warp frame and ExG mask onto canvas
        warped_rgb = cv2.warpPerspective(
            frame, self._H, (CANVAS_W, CANVAS_H),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )
        ones = np.full((h, w), 255, dtype=np.uint8)
        warped_hit = cv2.warpPerspective(
            ones, self._H, (CANVAS_W, CANVAS_H),
            flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT
        )
        warped_exg = cv2.warpPerspective(
            exg_mask(frame), self._H, (CANVAS_W, CANVAS_H),
            flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT
        )

        hit = warped_hit > 0
        self._canvas_rgb[hit] = warped_rgb[hit]
        self._canvas_exg = np.maximum(self._canvas_exg, warped_exg)
        self._canvas_hit[hit] = 255

    def render(self) -> np.ndarray | None:
        """Return a cropped composite: RGB mosaic + green weed overlay."""
        rows = np.any(self._canvas_hit, axis=1)
        cols = np.any(self._canvas_hit, axis=0)
        if not rows.any():
            return None

        r0, r1 = int(np.where(rows)[0][0]),  int(np.where(rows)[0][-1])
        c0, c1 = int(np.where(cols)[0][0]),  int(np.where(cols)[0][-1])
        pad = 40
        r0 = max(0, r0 - pad);  r1 = min(CANVAS_H, r1 + pad)
        c0 = max(0, c0 - pad);  c1 = min(CANVAS_W, c1 + pad)

        rgb = self._canvas_rgb[r0:r1, c0:c1].copy()
        exg = self._canvas_exg[r0:r1, c0:c1]

        # Semi-transparent green fill over weed areas
        tint = np.zeros_like(rgb)
        tint[exg > 0] = (0, 180, 0)
        out = cv2.addWeighted(rgb, 1.0, tint, 0.45, 0)

        # Bright green contour around each weed blob
        contours, _ = cv2.findContours(exg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 60), 1)

        return out

    def save(self, path: str | None = None) -> str:
        img = self.render()
        if img is None:
            print("No frames processed yet — nothing to save.", flush=True)
            return ''
        if path is None:
            path = f"weed_map_{int(time.time())}.png"
        cv2.imwrite(path, img)
        print(f"Saved → {path}  ({img.shape[1]}×{img.shape[0]} px)", flush=True)
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
    else:
        cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            cap = cv2.VideoCapture(FFMPEG_URL, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Failed to open video source.", flush=True)
        sys.exit(1)

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if offline else 0
    print(f"{'Offline' if offline else 'Live'} source open"
          + (f" — {total} frames" if total > 0 else ""), flush=True)

    builder = MosaicBuilder()
    WIN = "Weed Map  [s=save  q=quit]"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 900, 900)

    raw_count = 0
    PROCESS_EVERY = 2   # only add every Nth frame — enough overlap, faster processing
    DISPLAY_EVERY = 20  # refresh the map window every N processed frames

    while True:
        ok, frame = cap.read()
        if not ok:
            if offline:
                break
            continue

        raw_count += 1
        if raw_count % PROCESS_EVERY != 0:
            continue

        builder.add_frame(frame)

        if builder.frame_count % DISPLAY_EVERY == 0:
            img = builder.render()
            if img is not None:
                h, w = img.shape[:2]
                scale = min(880 / w, 880 / h)
                disp = cv2.resize(img, (int(w * scale), int(h * scale)),
                                  interpolation=cv2.INTER_AREA)
                if offline and total > 0:
                    pct = int(100 * raw_count / total)
                    cv2.putText(disp, f"{pct}%  ({builder.frame_count} frames)",
                                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (200, 200, 200), 1, cv2.LINE_AA)
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
