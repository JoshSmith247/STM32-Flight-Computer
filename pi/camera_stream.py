#!/usr/bin/env python3
"""
H.264 camera stream: Arducam OV9782 (USB/UVC) → MPEG-TS/UDP to GCS laptop.

The OV9782 is a USB UVC camera — it appears as /dev/video0 (or similar) and
outputs MJPEG, which ffmpeg captures and (re)encodes for the link.

IMPORTANT — the encoder is the #1 cause of latency/CPU-saturation on a Pi Zero 2 W:
    ENCODER=hw    (default)  hardware H.264 via h264_v4l2m2m  ← offloads the CPU
    ENCODER=sw               software H.264 via libx264       ← CPU-bound, laggy/hangs
                                                                 a Zero 2 W at high res
    ENCODER=copy             relay the camera's MJPEG, no transcode (lowest CPU/latency,
                                                                 higher bandwidth)

SAFE FALLBACK: when ENCODER=hw, this script verifies `h264_v4l2m2m` actually exists
(and that ffmpeg doesn't immediately fail at runtime). If the hardware encoder is
missing or won't open, it automatically falls back to software at a *low* resolution
(SW_FALLBACK_WIDTH x SW_FALLBACK_HEIGHT, default 640x480) instead of letting systemd
crash-loop the service. Install/enable the HW encoder to get full resolution back.

Install on Pi:        sudo apt install ffmpeg
Check HW encoder:     ffmpeg -hide_banner -encoders | grep h264_v4l2m2m

Receive on laptop (hw/sw → mpegts H.264) — add low-latency flags:
    ffplay -fflags nobuffer -flags low_delay -framedrop udp://@0.0.0.0:5600
Receive on laptop (copy → MJPEG): prefer GStreamer:
    gst-launch-1.0 udpsrc port=5600 ! tsdemux ! jpegdec ! autovideosink

Usage:
    python3 camera_stream.py [LAPTOP_IP]
    LAPTOP_IP env var also accepted (default 192.168.4.2).
"""

import os
import sys
import time
import signal
import subprocess

LAPTOP_IP  = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '192.168.4.2')
VIDEO_PORT = int(os.environ.get('VIDEO_PORT', '5600'))
WIDTH      = int(os.environ.get('CAM_WIDTH',  '1280'))
HEIGHT     = int(os.environ.get('CAM_HEIGHT', '800'))
FRAMERATE  = int(os.environ.get('CAM_FPS',    '30'))
CAM_DEVICE = os.environ.get('CAM_DEVICE', '/dev/video0')
ENCODER    = os.environ.get('ENCODER', 'hw').lower()
BITRATE    = os.environ.get('CAM_BITRATE', '2M')
SW_FB_W    = int(os.environ.get('SW_FALLBACK_WIDTH',  '640'))
SW_FB_H    = int(os.environ.get('SW_FALLBACK_HEIGHT', '480'))


def encoder_available(name: str) -> bool:
    """True if ffmpeg lists the given encoder (e.g. 'h264_v4l2m2m')."""
    try:
        out = subprocess.run(
            ['ffmpeg', '-hide_banner', '-encoders'],
            capture_output=True, text=True, timeout=10,
        ).stdout
    except (FileNotFoundError, subprocess.SubprocessError):
        return False
    return name in out


def build_cmd(encoder: str, width: int, height: int) -> list:
    """Assemble the ffmpeg pipeline for the selected encoder + resolution."""
    capture = [
        'ffmpeg',
        '-hide_banner', '-loglevel', 'warning', '-nostats',  # no per-frame stats
                                                             # spamming journald/SD
        '-f', 'v4l2',
        '-input_format', 'mjpeg',
        '-framerate', str(FRAMERATE),
        '-video_size', f'{width}x{height}',
        '-i', CAM_DEVICE,
    ]

    if encoder == 'hw':
        # Hardware H.264 on the VideoCore (Pi Zero 2 W / Pi 3 / Pi 4). Frees the CPU,
        # which is what fixes the lag/hang. NOTE: h264_v4l2m2m does NOT accept
        # libx264's -preset/-tune flags — keep them off this branch.
        encode = ['-c:v', 'h264_v4l2m2m', '-b:v', BITRATE, '-g', str(FRAMERATE)]
    elif encoder == 'sw':
        # Software H.264 — CPU-bound; keep the resolution low on a Zero 2 W.
        encode = [
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-g', '15',
            '-b:v', '1M',
        ]
    elif encoder == 'copy':
        # Zero transcoding: relay the camera's MJPEG straight through.
        encode = ['-c:v', 'copy']
    else:
        print(f"Unknown ENCODER={encoder!r} (use hw | sw | copy)", flush=True)
        sys.exit(2)

    tail = [
        '-muxdelay', '0',
        '-muxpreload', '0',
        '-f', 'mpegts',
        f'udp://{LAPTOP_IP}:{VIDEO_PORT}?pkt_size=1316&buffer_size=65536',
    ]
    return capture + encode + tail


def run_ffmpeg(encoder: str, width: int, height: int) -> int:
    """Run ffmpeg to completion; return its exit code (-SIGTERM if we stopped it)."""
    cmd = build_cmd(encoder, width, height)
    print(f"Streaming {width}x{height}@{FRAMERATE}fps via ENCODER={encoder} "
          f"→ udp://{LAPTOP_IP}:{VIDEO_PORT} (mpegts)", flush=True)
    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("ffmpeg not found — install with: sudo apt install ffmpeg", flush=True)
        sys.exit(1)
    try:
        proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGTERM)
        proc.wait()
        raise
    return proc.returncode


def main() -> None:
    if not os.path.exists(CAM_DEVICE):
        print(f"Camera device not found: {CAM_DEVICE}", flush=True)
        sys.exit(1)

    encoder, width, height = ENCODER, WIDTH, HEIGHT

    # Gate 1: hw requested but the encoder isn't even listed → fall back up front.
    if encoder == 'hw' and not encoder_available('h264_v4l2m2m'):
        print(f"WARNING: h264_v4l2m2m (hardware H.264) not found in ffmpeg — falling "
              f"back to software libx264 at {SW_FB_W}x{SW_FB_H} to avoid CPU "
              f"saturation. Install/enable the HW encoder for full resolution.",
              flush=True)
        encoder, width, height = 'sw', SW_FB_W, SW_FB_H

    try:
        start = time.monotonic()
        rc = run_ffmpeg(encoder, width, height)
        elapsed = time.monotonic() - start

        # Gate 2: hw was listed but ffmpeg died almost immediately → the V4L2 M2M
        # device likely failed to open. Fall back once to software at low res rather
        # than letting systemd's Restart=always crash-loop us every few seconds.
        if encoder == 'hw' and rc not in (0, -signal.SIGTERM) and elapsed < 5.0:
            print(f"WARNING: hardware encoder failed at runtime (exit {rc} after "
                  f"{elapsed:.1f}s) — retrying once with software libx264 at "
                  f"{SW_FB_W}x{SW_FB_H}.", flush=True)
            rc = run_ffmpeg('sw', SW_FB_W, SW_FB_H)
    except KeyboardInterrupt:
        sys.exit(0)

    if rc not in (0, -signal.SIGTERM):
        print(f"ffmpeg exited with code {rc}", flush=True)
        sys.exit(rc)


if __name__ == '__main__':
    main()
