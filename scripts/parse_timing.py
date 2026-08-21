#!/usr/bin/env python3
"""
Parse per-component timing from a captured ROS2 launch log.

Usage:
    # Capture log while running (one condition):
    ros2 launch ov_msckf subscribe.launch.py config_path:=... 2>&1 | tee /tmp/timing_yolo.log
    # Then parse:
    python3 scripts/parse_timing.py /tmp/timing_yolo.log [--label "YOLO masked"]

Output: summary table with mean / std / max per component and % of 50ms frame budget.
"""

import re
import sys
import argparse
import numpy as np

FRAME_BUDGET_MS = 50.0  # 20 Hz camera → 50 ms per frame

# ── Pattern definitions ──────────────────────────────────────────────────────

PATTERNS = {
    # Python nodes (our instrumentation)
    'YOLO inference':       re.compile(r'\[TIMING\]\[yolo\] inference_ms=([\d.]+)'),
    'YOLO CB (cached pub)': re.compile(r'\[TIMING\]\[yolo_cb\] publish_ms=([\d.]+)'),
    'HSE (ORB+triangulate)':re.compile(r'\[TIMING\]\[hse\] callback_ms=([\d.]+)'),

    # OpenVINS INFO-level (always printed):
    # "[TIME]: 0.0523 seconds total (19.1 hz, 3.23 ms behind)"
    'OpenVINS total':       re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds total'),

    # OpenVINS DEBUG-level (verbosity: DEBUG):
    # "[TIME]: 0.0123 seconds for tracking"
    # "[TIME]: 0.0023 seconds for propagation"
    # "[TIME]: 0.0089 seconds for MSCKF update"
    # "[TIME]: 0.0034 seconds for SLAM update"
    # "[TIME]: 0.0001 seconds for SLAM delayed init"
    # "[TIME]: 0.0021 seconds for re-tri & marg"
    'OV tracking (KLT)':    re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for tracking'),
    'OV propagation':       re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for propagation'),
    'OV MSCKF update':      re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for MSCKF update'),
    'OV SLAM update':       re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for SLAM update'),
    'OV SLAM init':         re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for SLAM delayed init'),
    'OV marginalization':   re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for re-tri'),
}


OV_NAMES = {'OpenVINS total', 'OV tracking (KLT)', 'OV propagation',
            'OV MSCKF update', 'OV SLAM update', 'OV SLAM init', 'OV marginalization'}


_ANSI = re.compile(r'\033\[[0-9;]*m')


def parse_log(path: str) -> dict[str, list[float]]:
    data: dict[str, list[float]] = {k: [] for k in PATTERNS}
    with open(path, 'r', errors='replace') as f:
        for raw_line in f:
            line = _ANSI.sub('', raw_line)  # strip color codes
            for name, pat in PATTERNS.items():
                m = pat.search(line)
                if m:
                    val = float(m.group(1))
                    if name in OV_NAMES:
                        val *= 1000.0  # seconds → ms
                    data[name].append(val)
    return data


def print_table(data: dict[str, list[float]], label: str):
    print()
    print(f'{"=" * 72}')
    print(f'  Timing Summary  —  {label}')
    print(f'  Frame budget: {FRAME_BUDGET_MS:.0f} ms  (20 Hz camera)')
    print(f'{"=" * 72}')
    print(f'{"Component":<28} {"N":>5}  {"Mean":>8}  {"Std":>7}  {"Max":>8}  {"% budget":>9}')
    print(f'{"-" * 72}')

    GROUPS = [
        ('─ YOLO masker (separate process) ─', [
            'YOLO CB (cached pub)',
            'YOLO inference',
        ]),
        ('─ OpenVINS VIO thread ─', [
            'OpenVINS total',
            'OV tracking (KLT)',
            'OV propagation',
            'OV MSCKF update',
            'OV SLAM update',
            'OV SLAM init',
            'OV marginalization',
        ]),
        ('─ Hybrid speed estimator (separate process) ─', [
            'HSE (ORB+triangulate)',
        ]),
    ]

    for group_title, keys in GROUPS:
        print(f'\n  {group_title}')
        for k in keys:
            vals = data[k]
            n = len(vals)
            if n == 0:
                print(f'  {"  " + k:<26} {"–":>5}  {"–":>8}  {"–":>7}  {"–":>8}  {"–":>9}')
                continue
            arr = np.array(vals)
            mean, std, mx = arr.mean(), arr.std(), arr.max()
            pct = mean / FRAME_BUDGET_MS * 100.0
            print(f'  {"  " + k:<26} {n:>5}  {mean:>7.2f}ms  {std:>6.2f}  {mx:>7.2f}ms  {pct:>8.1f}%')

    print(f'\n{"=" * 72}')
    print()
    print('  Note: YOLO inference runs in a BACKGROUND THREAD — it does NOT')
    print('  add to OpenVINS frame latency. VIO always reads from YOLO cache.')
    print('  Only "OV total" is the real VIO latency figure for the paper.')
    print(f'{"=" * 72}')
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', help='Path to captured ROS2 log file')
    parser.add_argument('--label', default='', help='Condition label (e.g. "YOLO masked, workstation")')
    args = parser.parse_args()

    label = args.label or args.log
    data = parse_log(args.log)

    total = sum(len(v) for v in data.values())
    if total == 0:
        print(f'ERROR: No timing entries found in {args.log}')
        print('  Check that verbosity: DEBUG is set in estimator_config.yaml')
        print('  and that [TIMING] lines appear in the log.')
        sys.exit(1)

    print_table(data, label)


if __name__ == '__main__':
    main()
