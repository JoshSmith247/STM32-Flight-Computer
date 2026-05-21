#!/usr/bin/env python3
"""
H.264 camera stream: Arducam OV9782 (CSI) → RTP/UDP to GCS laptop.

Arducam OV9782 setup (once, in /boot/config.txt):
    dtoverlay=arducam-64mp     # or the specific OV9782 overlay from Arducam's install script
    # Run:  wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
    # Then: bash install_pivariety_pkgs.sh -p imx519   (adjust for OV9782 package name)

Receive on laptop (GStreamer):
    gst-launch-1.0 udpsrc port=5600 \
      caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
      ! rtph264depay ! avdec_h264 ! autovideosink

Or with OpenCV (in ground/inference.py, already handled there).

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

RPICAM_CMD = [
    'rpicam-vid',
    '-t', '0',
    '--width',     str(WIDTH),
    '--height',    str(HEIGHT),
    '--framerate', str(FRAMERATE),
    '--codec',     'h264',
    '--inline',    # embed SPS/PPS before every keyframe so late-joining receivers decode cleanly
    '--nopreview',
    '-o', '-',
]

GST_CMD = [
    'gst-launch-1.0', '-q',
    'fdsrc', '!',
    'h264parse', '!',
    'rtph264pay', 'config-interval=1', 'pt=96', '!',
    'udpsink', f'host={LAPTOP_IP}', f'port={VIDEO_PORT}',
]


def main() -> None:
    print(f"Streaming {WIDTH}x{HEIGHT}@{FRAMERATE}fps → udp://{LAPTOP_IP}:{VIDEO_PORT}", flush=True)

    cam = subprocess.Popen(RPICAM_CMD, stdout=subprocess.PIPE)
    gst = subprocess.Popen(GST_CMD, stdin=cam.stdout)
    cam.stdout.close()  # parent closes so gst gets EOF if cam dies

    try:
        cam.wait()
    except KeyboardInterrupt:
        cam.send_signal(signal.SIGTERM)
        gst.send_signal(signal.SIGTERM)
    finally:
        cam.wait()
        gst.wait()


if __name__ == '__main__':
    main()
