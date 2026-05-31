import math

import cv2
import numpy as np

import config
from mavlink import pixel_to_world


class WeedTracker:
    def __init__(self):
        self._tracks: dict = {}            # YOLO tracks (re-enable with YOLO section)
        self._prev_gray = None
        self._exg_boxes: list = []
        self._named: dict[int, dict] = {}  # weed_id → {box, template, pts, lost_frames, world_pos}
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
                self._prev_gray, gray, pts, None, **config.LK_PARAMS
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
        if len(w['templates']) > config.TEMPLATE_POOL_SIZE:
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

        # Pass 1 — optical flow predicts new position; ExG center check confirms it.
        # The predicted box is NOT written back unless its center lands inside a blob,
        # so a drifting flow can't silently claim an unrelated green patch.
        for wid, w in self._named.items():
            # Predict where the box moved this frame
            prop_box = w['box']
            pts = w['pts']
            if pts is not None and len(pts) >= 3:
                new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self._prev_gray, gray, pts, None, **config.LK_PARAMS
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

        # Pass 2 — re-identification for lost named weeds.
        # World position gates candidates to blobs within WORLD_REMATCH_DIST metres;
        # template matching then picks the best among those (graceful fallback when
        # no GPS fix is available: all blobs are considered).
        for wid, w in self._named.items():
            if w['lost_frames'] == 0:
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
                        if dist > config.WORLD_REMATCH_DIST:
                            continue   # too far in the world
                        world_confirmed = True
                # GPS-confirmed candidates need only a loose template sanity check;
                # without GPS, template matching is the only discriminator so we
                # require a stricter score to avoid cross-ID between similar weeds.
                thresh = config.REMATCH_THRESH_GPS if world_confirmed else config.REMATCH_THRESH
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
                if self._selected_wid == wid:
                    self._selected_wid = None
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
        if y < config.SIDEBAR_HDR_H:
            return
        idx  = (y - config.SIDEBAR_HDR_H) // config.SIDEBAR_ROW_H
        wids = sorted(self._named.keys())
        if 0 <= idx < len(wids):
            wid = wids[idx]
            self._selected_wid = None if self._selected_wid == wid else wid

    def draw_sidebar(self, h: int) -> np.ndarray:
        """Return a SIDEBAR_W × h panel listing all named weeds."""
        panel = np.full((h, config.SIDEBAR_W, 3), (35, 35, 35), dtype=np.uint8)

        # Title bar — same height, background and separator as the FLIGHT COMPUTER header
        cv2.rectangle(panel, (0, 0), (config.SIDEBAR_W, config.SIDEBAR_HDR_H - 1), (40, 44, 50), -1)
        cv2.putText(panel, "Named Weeds", (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (185, 192, 200), 1, cv2.LINE_AA)
        cv2.line(panel, (0, config.SIDEBAR_HDR_H - 1),
                 (config.SIDEBAR_W, config.SIDEBAR_HDR_H - 1), (60, 60, 60), 1)

        mid = config.SIDEBAR_ROW_H // 2
        for i, wid in enumerate(sorted(self._named.keys())):
            w        = self._named[wid]
            y0       = config.SIDEBAR_HDR_H + i * config.SIDEBAR_ROW_H
            active   = w['lost_frames'] == 0
            selected = wid == self._selected_wid

            # Row highlight for selected weed
            if selected:
                cv2.rectangle(panel, (2, y0 + 1),
                              (config.SIDEBAR_W - 2, y0 + config.SIDEBAR_ROW_H - 1),
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
                cv2.putText(panel, "GPS", (config.SIDEBAR_W - 38, y0 + mid + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 150, 80), 1, cv2.LINE_AA)

        return panel

    def centroids(self) -> list[tuple[float, float]]:
        result = []
        for track in self._tracks.values():
            x1, y1, x2, y2 = track['box']
            result.append(((x1 + x2) / 2, (y1 + y2) / 2))
        return result
