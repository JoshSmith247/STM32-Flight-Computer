#!/usr/bin/env python3
"""
Standalone YOLO inference worker — runs as a subprocess of the GCS.

Protocol (binary over stdin/stdout):
  stdin:  12-byte header (h, w, c as 3×int32 little-endian) + raw uint8 frame bytes
  stdout: one JSON line per inference result, e.g. [[x1,y1,x2,y2,conf], …]
          First line is the literal string "ready" once the model is loaded.
  stderr: passed through to terminal (YOLO progress, debug output)

Running in a separate process means this code has its own Metal context that
is completely isolated from DearPyGUI's Metal render thread in the main process,
eliminating the SIGSEGV that occurs when PyTorch and DPG share Metal from
different threads.
"""

import json
import os
import signal
import struct
import sys

# Restore default SIGPIPE handling so the process exits silently (no traceback)
# if the parent closes the pipe before we finish initializing.
try:
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
except AttributeError:
    pass  # Windows

os.environ.update({
    'OMP_NUM_THREADS':        '1',
    'VECLIB_MAXIMUM_THREADS': '1',
    'MKL_NUM_THREADS':        '1',
    'OPENBLAS_NUM_THREADS':   '1',
    'PYTORCH_ENABLE_MPS_FALLBACK': '1',
})

model_path = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n.pt'

try:
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)

    from ultralytics import YOLO
    import numpy as np

    # Handle corrupt or missing model file
    if not os.path.isfile(model_path):
        bare = os.path.basename(model_path)
        print(f"[yolo_worker] weights not found at {model_path!r} — downloading {bare!r}",
              file=sys.stderr, flush=True)
        model_path = bare

    # Redirect fd 1 → fd 2 (stderr) during init so ultralytics' logging
    # StreamHandler (which holds the original stdout fd) doesn't write
    # the fuse summary to the protocol pipe before we send 'ready'.
    _saved_fd1 = os.dup(1)
    os.dup2(2, 1)
    try:
        try:
            model = YOLO(model_path)
        except RuntimeError as exc:
            if 'zip' in str(exc).lower() or 'central directory' in str(exc).lower():
                print(f"[yolo_worker] corrupt model, re-downloading", file=sys.stderr, flush=True)
                if os.path.isfile(model_path):
                    os.remove(model_path)
                model = YOLO(os.path.basename(model_path))
            else:
                raise
        model.fuse()
    finally:
        os.dup2(_saved_fd1, 1)
        os.close(_saved_fd1)

except Exception as exc:
    print(f"[yolo_worker] init failed: {exc}", file=sys.stderr, flush=True)
    sys.exit(1)

# Signal the parent that we're ready
sys.stdout.write('ready\n')
sys.stdout.flush()

stdin_b  = sys.stdin.buffer
stdout_b = sys.stdout.buffer

while True:
    header = stdin_b.read(12)
    if len(header) < 12:
        break   # parent closed stdin → exit cleanly

    h, w, c = struct.unpack('<iii', header)
    n_bytes  = h * w * c
    raw      = b''
    while len(raw) < n_bytes:
        chunk = stdin_b.read(n_bytes - len(raw))
        if not chunk:
            sys.exit(0)
        raw += chunk

    frame = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, c)

    try:
        with torch.inference_mode():
            results = model.predict(frame, classes=[0], verbose=False,
                                    conf=0.40, device='cpu')
        boxes = []
        for r in results:
            for b in r.boxes:
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                boxes.append([x1, y1, x2, y2, float(b.conf[0])])
    except Exception as exc:
        print(f"[yolo_worker] inference error: {exc}", file=sys.stderr, flush=True)
        boxes = []

    line = json.dumps(boxes).encode() + b'\n'
    stdout_b.write(line)
    stdout_b.flush()
