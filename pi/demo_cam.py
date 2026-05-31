#!/usr/bin/env python3
"""
Demo camera stream: loops pi/demo/active.mp4 as RTP/H.264/UDP, matching
the format that camera_stream.py sends from the real Arducam on the Pi.

Use this on your laptop to test inference.py end-to-end without hardware.
Both scripts should target 127.0.0.1 when run locally.

Requires: ffmpeg in PATH  (brew install ffmpeg)
"""

import os
import sys
import subprocess

LAPTOP_IP  = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '127.0.0.1')
VIDEO_PORT = int(os.environ.get('VIDEO_PORT', '5600'))
VIDEO_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'demo', 'active.mp4')

FFMPEG_CMD = [
    'ffmpeg',
    '-re',                  # play at real-time rate
    '-stream_loop', '-1',   # loop indefinitely
    '-i', VIDEO_FILE,
    '-an',                   # drop audio
    '-vcodec', 'libx264',    # re-encode to H.264 (handles any source codec)
    '-preset', 'ultrafast',
    '-tune', 'zerolatency',
    '-g', '15',              # keyframe every 15 frames, decoder syncs within 0.5 s
    '-f', 'mpegts',
    f'udp://{LAPTOP_IP}:{VIDEO_PORT}',
]


def main() -> None:
    if not os.path.exists(VIDEO_FILE):
        print(f"Video not found: {VIDEO_FILE}", flush=True)
        print(f"Place your test video at: {VIDEO_FILE}", flush=True)
        sys.exit(1)

    print(f"Demo stream: active.mp4 → udp://{LAPTOP_IP}:{VIDEO_PORT} (mpegts, looping)", flush=True)

    proc = subprocess.Popen(FFMPEG_CMD)
    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.terminate()
        proc.wait()


if __name__ == '__main__':
    main()
