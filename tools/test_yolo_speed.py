"""
Quick sanity check: YOLO inference speed and device for yolo_masker.
Pass/fail based on 20Hz stereo budget (50ms per stereo pair).
"""
import time
import numpy as np
from ultralytics import YOLO

CLASSES = [0, 1, 2, 3, 5, 6, 7]
CONF    = 0.25
H, W    = 480, 752
N_RUNS  = 20
BUDGET_MS = 50.0   # 20 Hz = one stereo pair every 50 ms

model = YOLO('yolo11n.pt')
model.to('cuda')  # optional
img   = np.random.randint(0, 255, (H, W, 3), dtype=np.uint8)

print(f"Device : {model.device}")
print(f"Image  : {W}×{H}")
print(f"Budget : {BUDGET_MS:.0f} ms per stereo pair  ({N_RUNS} warmup+timed runs)\n")

# Warmup (first inference allocates CUDA memory)
for _ in range(3):
    model(img, classes=CLASSES, conf=CONF, verbose=False)

# Timed runs — simulate one stereo pair (cam0 + cam1) per iteration
times_pair = []
for _ in range(N_RUNS):
    t0 = time.perf_counter()
    model(img, classes=CLASSES, conf=CONF, verbose=False)   # cam0
    model(img, classes=CLASSES, conf=CONF, verbose=False)   # cam1
    times_pair.append((time.perf_counter() - t0) * 1000)

mean_ms = sum(times_pair) / len(times_pair)
max_ms  = max(times_pair)

print(f"Stereo pair  mean: {mean_ms:.1f} ms   max: {max_ms:.1f} ms")
print(f"Single camera     mean: {mean_ms/2:.1f} ms")
print()
if mean_ms < BUDGET_MS:
    print(f"PASS — mean {mean_ms:.1f} ms < {BUDGET_MS:.0f} ms budget. Safe to run experiments.")
else:
    print(f"FAIL — mean {mean_ms:.1f} ms > {BUDGET_MS:.0f} ms budget. Inference still too slow.")
