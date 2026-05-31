#!/usr/bin/env python3
"""
H.264 camera stream: Arducam OV9782 (USB/UVC) → RTP/UDP to GCS laptop.

The OV9782 is a USB UVC camera — it appears as /dev/video0 (or similar).
Uses ffmpeg v4l2 capture + libx264 encoding in a single process.

Install on Pi:
    sudo apt install ffmpeg

Receive on laptop:
    ffplay rtp://0.0.0.0:5600
    # or GStreamer:
    gst-launch-1.0 udpsrc port=5600 \
      caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
      ! rtph264depay ! avdec_h264 ! autovideosink

Usage:
    python3 camera_stream.py [LAPTOP_IP]
    LAPTOP_IP env var also accepted (default 192.168.4.2).
"""

import os
import sys
import signal
import subprocess

LAPTOP_IP  = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '192.168.4.2')
VIDEO_PORT = int(os.environ.get('VIDEO_PORT', '5600'))
WIDTH      = int(os.environ.get('CAM_WIDTH',  '1280'))
HEIGHT     = int(os.environ.get('CAM_HEIGHT', '800'))
FRAMERATE  = int(os.environ.get('CAM_FPS',    '30'))
CAM_DEVICE = os.environ.get('CAM_DEVICE', '/dev/video0')


def main() -> None:
    cmd = [
        'ffmpeg',
        '-f', 'v4l2',
        '-input_format', 'mjpeg',
        '-framerate', str(FRAMERATE),
        '-video_size', f'{WIDTH}x{HEIGHT}',
        '-i', CAM_DEVICE,
        '-c:v', 'libx264',
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-g', '15',
        '-b:v', '1M',
        '-muxdelay', '0',
        '-muxpreload', '0',
        '-f', 'mpegts',
        f'udp://{LAPTOP_IP}:{VIDEO_PORT}?pkt_size=1316&buffer_size=65536',
    ]

    if not os.path.exists(CAM_DEVICE):
        print(f"Camera device not found: {CAM_DEVICE}", flush=True)
        sys.exit(1)

    print(f"Streaming {WIDTH}x{HEIGHT}@{FRAMERATE}fps → udp://{LAPTOP_IP}:{VIDEO_PORT} (mpegts)", flush=True)

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("ffmpeg not found — install with: sudo apt install ffmpeg", flush=True)
        sys.exit(1)

    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGTERM)
    finally:
        proc.wait()

    if proc.returncode not in (0, -signal.SIGTERM):
        print(f"ffmpeg exited with code {proc.returncode}", flush=True)
        sys.exit(proc.returncode)


if __name__ == '__main__':
    main()
