import cv2
import numpy as np

import config


def exg_mask(frame: np.ndarray) -> np.ndarray:
    """Return a binary mask of vegetation pixels.

    Two gates must both pass:
      1. Excess Green: 2G - R - B > EXG_THRESH  (colour arithmetic)
      2. HSV saturation >= EXG_SAT_MIN           (rocks are grey/desaturated; weeds aren't)
    """
    f = frame.astype(np.float32)
    exg = 2.0 * f[:, :, 1] - f[:, :, 2] - f[:, :, 0]
    exg_gate = (exg > config.EXG_THRESH).astype(np.uint8)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Hue 25-85 (OpenCV 0-179 scale) covers yellow-green -> green -> blue-green
    # Saturation floor rejects grey rocks, concrete, and lichen
    sat_gate = cv2.inRange(hsv, (25, config.EXG_SAT_MIN, 30), (85, 255, 255))

    binary = (exg_gate * sat_gate)   # both conditions must hold
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary


def exg_candidates(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Return (x1, y1, x2, y2) bounding boxes for each green blob within the
    allowed size range, with boxes fully contained inside a larger box removed."""
    frame_area = mask.shape[0] * mask.shape[1]
    max_area   = config.EXG_MAX_FRAC * frame_area

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if config.EXG_MIN_AREA <= area <= max_area:
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
