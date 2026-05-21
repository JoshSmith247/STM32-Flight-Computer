#!/usr/bin/env python3
"""
Ground station: receive H.264 stream from Pi, run YOLO weed detection,
send MAVLink targeting commands back to the drone.

Requires OpenCV built with GStreamer (standard on Ubuntu/Pi OS).
On macOS: brew install opencv or build from source with -DWITH_GSTREAMER=ON.
"""

import os
import sys
import socket
import cv2
from ultralytics import YOLO
from dotenv import load_dotenv

load_dotenv()

LAPTOP_IP   = os.environ.get('LAPTOP_IP',   '192.168.4.2')
VIDEO_PORT  = int(os.environ.get('VIDEO_PORT',  '5600'))
GCS_PORT    = int(os.environ.get('GCS_PORT',    '14550'))
PI_IP       = os.environ.get('PI_IP',       '192.168.4.1')
MODEL_PATH  = os.environ.get('MODEL_PATH',  'yolov8n.pt')  # replace with trained weed model
CONF_THRESH = float(os.environ.get('CONF_THRESH', '0.5'))

GST_PIPELINE = (
    f"udpsrc port={VIDEO_PORT} "
    f"caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
    f"! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1"
)


def send_target(sock: socket.socket, x_norm: float, y_norm: float) -> None:
    # Normalized target offset from frame center: x/y in [-1.0, 1.0].
    # Full MAVLink SET_POSITION_TARGET_LOCAL_NED framing goes here once
    # navigation_task is built out on the STM32 side.
    print(f"Target offset: x={x_norm:+.2f} y={y_norm:+.2f}", flush=True)
    # TODO: sock.sendto(mavlink_frame, (PI_IP, GCS_PORT))


def main() -> None:
    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open video stream — check GStreamer install and that Pi is streaming", flush=True)
        sys.exit(1)

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Stream open: {frame_w}x{frame_h} — running inference", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        results = model(frame, conf=CONF_THRESH, verbose=False)

        for box in results[0].boxes:
            cx = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
            cy = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
            x_norm = (cx - frame_w / 2) / (frame_w / 2)
            y_norm = (cy - frame_h / 2) / (frame_h / 2)
            send_target(sock, x_norm, y_norm)

        cv2.imshow("Weed Detection", results[0].plot())
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == '__main__':
    main()
