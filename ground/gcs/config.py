import os
import pathlib

import cv2
from dotenv import load_dotenv

load_dotenv()

LAPTOP_IP         = os.environ.get('LAPTOP_IP',        '192.168.4.2')
VIDEO_PORT        = int(os.environ.get('VIDEO_PORT',      '5600'))
# VIDEO_SOURCE: override the UDP stream with a local source for development.
#   ""          → default: UDP stream from Pi (or demo_cam.py)
#   "0","1"…   → system webcam by index
#   "/path/…"  → local video file (loops automatically)
VIDEO_SOURCE      = os.environ.get('VIDEO_SOURCE',     '')
GCS_PORT          = int(os.environ.get('GCS_PORT',        '14550'))
PI_IP             = os.environ.get('PI_IP',            '192.168.4.1')
WEED_TARGET_PORT  = int(os.environ.get('WEED_TARGET_PORT', '5700'))
EXG_THRESH        = float(os.environ.get('EXG_THRESH',  '20.0'))
EXG_MIN_AREA      = max(1,     int(os.environ.get('EXG_MIN_AREA',  '250')))
EXG_SAT_MIN       = max(0, min(255, int(os.environ.get('EXG_SAT_MIN', '40'))))
EXG_MAX_FRAC      = max(1e-4, min(0.99, float(os.environ.get('EXG_MAX_FRAC', '0.04'))))
YOLO_MODEL       = os.environ.get('YOLO_MODEL',
                       str(pathlib.Path(__file__).parent.parent / 'models' / 'yolov8n.pt'))
FOLLOW_SEND_HZ   = float(os.environ.get('FOLLOW_SEND_HZ', '3.0'))  # position update rate in Follow Me

REMATCH_THRESH     = 0.35    # template correlation floor — no GPS
REMATCH_THRESH_GPS = 0.12    # template correlation floor — GPS world-gate already fired
TEMPLATE_POOL_SIZE = 3       # rolling window of templates kept per weed
MAX_LOST_FRAMES    = 90      # frames (~3 s at 30 fps) before a lost named weed is discarded
CAM_HFOV           = float(os.environ.get('CAM_HFOV',           '62.0'))  # camera horizontal FOV, degrees
WORLD_REMATCH_DIST = float(os.environ.get('WORLD_REMATCH_DIST',  '0.4'))  # metres radius for position-based re-ID
CRUISE_ALT         = float(os.environ.get('CRUISE_ALT',          '10.0')) # target cruise altitude above ground, metres

GST_PIPELINE = (
    f"udpsrc port={VIDEO_PORT} "
    f"! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=1"
)
FFMPEG_URL = f"udp://@0.0.0.0:{VIDEO_PORT}?overrun_nonfatal=1&fifo_size=5000000&timeout=5000000"

LK_PARAMS = dict(winSize=(21, 21), maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Sidebar dimensions (drawn at display resolution, appended to the right of the frame)
SIDEBAR_W      = 180
SIDEBAR_HDR_H  = 37   # title bar height — matches FLIGHT COMPUTER header (36 px + 1 px separator)
SIDEBAR_ROW_H  = 28   # height per weed entry
STATS_W        = 1800  # drone stats panel width (2-column) — mutated by main() after window probe
PR             = 1     # display pixel ratio (1 = standard, 2 = Retina) — set by main()
PROGRAMS       = ['MANUAL', 'FOLLOW ME', 'WEED PICKER', 'RETURN HOME']
PROG_MANUAL    = 0
PROG_FOLLOW_ME = 1
PROG_WEED_PICK = 2
PROG_RTH       = 3

# Payload flag bit → display name.  Must match payload_flags constants in src/types.rs.
PAYLOAD_NAMES = {
    0: 'Servo Bus (4ch)',
}

OVERLAY_H = 28 + len(PROGRAMS) * 26 + 10 + 30 + 8

# Env defaults — these are also mutated at runtime by the overlay settings panel.
RECORD_VIDEO    = bool(int(os.environ.get('RECORD_VIDEO',    '0')))
RECORD_ACTIVE   = RECORD_VIDEO   # runtime toggle; overlay writes this
VERBOSE_LOGGING = bool(int(os.environ.get('VERBOSE_LOGGING', '0')))
