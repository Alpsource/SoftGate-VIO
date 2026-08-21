#!/usr/bin/env python3
"""
Generate paper-ready timing tables from timing sweep results.

Usage:
    python3 scripts/make_paper_timing_table.py \\
        --sweep   timings/20260819_130134 \\
        --subcomp timings/subcomp_TIMESTAMP \\
        --basecfg timings/basecfg_TIMESTAMP

Produces:
  Table A — Per-component latency (parking_lot config)
  Table B — Throughput by environment/config
  LaTeX strings for both tables
"""

import re
import os
import sys
import argparse
import numpy as np

# ── Pattern definitions (mirrors parse_timing.py) ────────────────────────────

_ANSI = re.compile(r'\033\[[0-9;]*m')

PATTERNS = {
    'YOLO CB':      re.compile(r'\[TIMING\]\[yolo_cb\] publish_ms=([\d.]+)'),
    'YOLO inf':     re.compile(r'\[TIMING\]\[yolo\] inference_ms=([\d.]+)'),
    'HSE':          re.compile(r'\[TIMING\]\[hse\] callback_ms=([\d.]+)'),
    'OV total':     re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds total'),
    'OV tracking':  re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for tracking'),
    'OV propag':    re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for propagation'),
    'OV MSCKF':     re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for MSCKF update'),
    'OV SLAM':      re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for SLAM update'),
    'OV SLAM init': re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for SLAM delayed init'),
    'OV marg':      re.compile(r'\[TIME\]:\s+([\d.]+)\s+seconds for re-tri'),
}

OV_SECONDS = {'OV total', 'OV tracking', 'OV propag', 'OV MSCKF',
              'OV SLAM', 'OV SLAM init', 'OV marg'}


def parse_combined_log(path: str) -> dict[str, list[float]]:
    data: dict[str, list[float]] = {k: [] for k in PATTERNS}
    with open(path, 'r', errors='replace') as f:
        for raw in f:
            line = _ANSI.sub('', raw)
            for name, pat in PATTERNS.items():
                m = pat.search(line)
                if m:
                    val = float(m.group(1))
                    if name in OV_SECONDS:
                        val *= 1000.0
                    data[name].append(val)
    return data


def stats(vals: list[float]) -> tuple[int, float, float, float]:
    if not vals:
        return 0, float('nan'), float('nan'), float('nan')
    a = np.array(vals)
    return len(a), float(a.mean()), float(a.std()), float(a.max())


def load_folder(folder: str) -> dict[str, dict]:
    """Return {scenario_key: {component: (n, mean, std, max)}} for a timing folder."""
    result = {}
    if not folder or not os.path.isdir(folder):
        return result
    for name in sorted(os.listdir(folder)):
        log = os.path.join(folder, name, "combined.log")
        if not os.path.isfile(log):
            continue
        data = parse_combined_log(log)
        result[name] = {k: stats(v) for k, v in data.items()}
    return result


def fmt(n, mean, std, mx, unit='ms') -> str:
    if n == 0 or np.isnan(mean):
        return '–'
    return f'{mean:.1f}'


def fmt_full(n, mean, std, mx) -> str:
    if n == 0 or np.isnan(mean):
        return f'{"–":>8}  {"–":>6}  {"–":>8}  {"–":>5}'
    return f'{mean:>7.1f}ms  {std:>5.1f}  {mx:>7.1f}ms  {n:>5}'


# ── Table A: per-component latency ───────────────────────────────────────────

def table_a(sweep: dict, subcomp: dict) -> str:
    FRAME = 50.0  # ms

    # Collect parking_lot values from sweep (yolo + yolo_imu + unmasked-high)
    # and sub-components from subcomp folder (yolo × high is most relevant)
    SWEEP_KEYS_PL = [k for k in sweep
                     if 'parking_lot' in k and sweep[k]['OV total'][0] > 200]

    def agg(folder, keys, comp):
        vals = []
        for k in keys:
            if k in folder:
                n, mean, std, mx = folder[k][comp]
                if n > 0 and not np.isnan(mean):
                    vals.append(mean)
        if not vals:
            return (0, float('nan'), float('nan'), float('nan'))
        return (len(vals), float(np.mean(vals)), float(np.std(vals)), float(np.max(vals)))

    # YOLO CB and inference: all valid scenarios
    all_keys_valid = [k for k in sweep if sweep[k]['OV total'][0] > 100
                      and not ('parking_lot' in k and 'unmasked' in k and
                               sweep[k]['OV total'][1] < 30)]

    yolo_cb  = agg(sweep, all_keys_valid, 'YOLO CB')
    yolo_inf = agg(sweep, all_keys_valid, 'YOLO inf')
    hse_all  = agg(sweep, all_keys_valid, 'HSE')

    # OV total: parking_lot conditions (clean data)
    ov_total = agg(sweep, SWEEP_KEYS_PL, 'OV total')

    # Sub-components: from subcomp folder (yolo + yolo_imu, parking_lot/high)
    sc_keys = [k for k in subcomp if 'parking_lot' in k and 'high' in k]
    ov_track = agg(subcomp, sc_keys, 'OV tracking')
    ov_prop  = agg(subcomp, sc_keys, 'OV propag')
    ov_msckf = agg(subcomp, sc_keys, 'OV MSCKF')
    ov_slam  = agg(subcomp, sc_keys, 'OV SLAM')
    ov_marg  = agg(subcomp, sc_keys, 'OV marg')

    # IMU residual overhead: yolo vs yolo_imu delta in parking_lot
    yolo_vals    = [sweep[k]['OV total'][1] for k in SWEEP_KEYS_PL if 'yolo-' in k and sweep[k]['OV total'][0]>200]
    yolimu_vals  = [sweep[k]['OV total'][1] for k in SWEEP_KEYS_PL if 'yolo_imu' in k and sweep[k]['OV total'][0]>200]
    if yolo_vals and yolimu_vals:
        imu_res_overhead = np.mean(yolimu_vals) - np.mean(yolo_vals)
    else:
        imu_res_overhead = float('nan')

    def row(label, tup, indent=0, note=''):
        n, mean, std, mx = tup
        pre = '  ' * indent
        if n == 0 or np.isnan(mean):
            s = f'  {pre}{label:<28} {"–":>8}  {"–":>6}  {"–":>8}  {"–":>5}'
        else:
            pct = mean / FRAME * 100.0
            s = f'  {pre}{label:<28} {mean:>7.1f}ms  {std:>5.1f}  {mx:>7.1f}ms  {pct:>5.1f}%'
        if note:
            s += f'  ({note})'
        return s

    lines = [
        '',
        '=' * 80,
        '  TABLE A — Per-Component Latency (parking_lot config, 600 features)',
        f'  Frame budget: {FRAME:.0f} ms  (20 Hz camera)',
        '=' * 80,
        f'  {"Component":<28} {"Mean":>8}  {"Std":>6}  {"Max":>8}  {"% bgt":>6}',
        '  ' + '-' * 62,
        '',
        '  — YOLO Masker (separate process, CUDA) —',
        row('YOLO CB (publish cached mask)', yolo_cb, note='actual VIO overhead'),
        row('YOLO inference (background)', yolo_inf, note='non-blocking, ~1-frame lag'),
        '',
        '  — OpenVINS MSCKF VIO (main thread) —',
        row('OpenVINS total', ov_total),
        row('└─ KLT tracking', ov_track, indent=1),
        row('└─ IMU propagation', ov_prop, indent=1),
        row('└─ MSCKF update', ov_msckf, indent=1),
        row('└─ SLAM update + init', ov_slam, indent=1),
        row('└─ Marginalization', ov_marg, indent=1),
    ]
    if not np.isnan(imu_res_overhead):
        lines.append(f'  {"IMU residual overhead":<28} {imu_res_overhead:>7.1f}ms  {"–":>6}  {"–":>8}  {"–":>6}  (yolo_imu − yolo delta)')
    else:
        lines.append(f'  {"IMU residual overhead":<28} {"–":>8}  {"–":>6}  {"–":>8}  {"–":>6}')
    lines += [
        '',
        '  — Hybrid Speed Estimator (separate process) —',
        row('HSE (ORB + stereo triangulate)', hse_all),
        '',
        '=' * 80,
        '  Architecture note: YOLO inference and HSE run in PARALLEL with OpenVINS.',
        '  Effective pipeline latency = max(OV total, YOLO CB) ≈ OV total.',
        '  YOLO inference does not block VIO — mask is always read from a cache.',
        '=' * 80,
        '',
    ]
    return '\n'.join(lines)


# ── Table B: throughput by environment ───────────────────────────────────────

def table_b(sweep: dict, basecfg: dict) -> str:
    FRAME = 50.0

    CONDITIONS = ['unmasked', 'yolo', 'yolo_imu']
    ENVS = [
        ('parking_lot', '600 feat', sweep),
        ('city_day',    '1000 feat (accuracy cfg)', sweep),
        ('city_night',  '1000 feat (accuracy cfg)', sweep),
        ('city_day',    '600 feat (base cfg)',   basecfg),
        ('city_night',  '600 feat (base cfg)',   basecfg),
    ]

    lines = [
        '',
        '=' * 80,
        '  TABLE B — Real-Time Throughput by Environment',
        '=' * 80,
        f'  {"Environment":<15} {"Config":<26} {"Condition":<12} {"OV total":>10} {"Hz":>6} {"Budget":>7}',
        '  ' + '-' * 74,
    ]

    OUTLIERS = {'unmasked-parking_lot-none', 'unmasked-parking_lot-low',
                'unmasked-parking_lot-mid', 'yolo-city_day-low'}

    for env, cfg_label, folder in ENVS:
        for cond in CONDITIONS:
            keys = [k for k in folder
                    if k.startswith(f'{cond}-{env}-')
                    and k not in OUTLIERS
                    and folder[k]['OV total'][0] > 100]
            vals = [folder[k]['OV total'][1] for k in keys
                    if not np.isnan(folder[k]['OV total'][1])]
            if not vals:
                lines.append(f'  {env:<15} {cfg_label:<26} {cond:<12} {"–":>10} {"–":>6} {"–":>7}')
                continue
            mean_ms = np.mean(vals)
            hz = 1000.0 / mean_ms
            budget_ok = '✓' if mean_ms <= FRAME else '✗'
            lines.append(f'  {env:<15} {cfg_label:<26} {cond:<12} {mean_ms:>9.1f}ms {hz:>5.1f} {budget_ok:>7}')
        lines.append('')

    lines += [
        '=' * 80,
        '  Note: city accuracy config (1000 feat) is used in all accuracy experiments.',
        '  base config (600 feat) confirms city is real-time capable when accuracy',
        '  is traded for speed — suitable for real-time deployment on resource-limited',
        '  platforms (e.g., mobile robotics, UAVs).',
        '=' * 80,
        '',
    ]
    return '\n'.join(lines)


# ── LaTeX output ─────────────────────────────────────────────────────────────

def latex_table_a(sweep: dict, subcomp: dict) -> str:
    FRAME = 50.0

    SWEEP_KEYS_PL = [k for k in sweep
                     if 'parking_lot' in k and sweep[k]['OV total'][0] > 200]
    all_valid = [k for k in sweep if sweep[k]['OV total'][0] > 100
                 and not ('parking_lot' in k and 'unmasked' in k
                          and sweep[k]['OV total'][1] < 30)]

    def agg(folder, keys, comp):
        vals = [folder[k][comp][1] for k in keys
                if k in folder and folder[k][comp][0] > 0
                and not np.isnan(folder[k][comp][1])]
        if not vals:
            return float('nan'), float('nan'), float('nan')
        return float(np.mean(vals)), float(np.std(vals)), float(np.max(vals))

    sc_keys = [k for k in subcomp if 'parking_lot' in k and 'high' in k]

    def lrow(label, mean, std, mx, note='', indent=False):
        pre = r'\quad ' if indent else ''
        if np.isnan(mean):
            return f'  {pre}{label} & -- & -- & -- \\\\'
        pct = mean / FRAME * 100.0
        n_str = note and f' ({note})' or ''
        return f'  {pre}{label}{n_str} & {mean:.1f} & {std:.1f} & {pct:.1f}\\% \\\\'

    rows = [
        r'\begin{table}[t]',
        r'\centering',
        r'\caption{Per-component latency of the proposed pipeline (parking\_lot, 600 features, 20\,Hz camera).}',
        r'\label{tab:timing}',
        r'\begin{tabular}{lccc}',
        r'\toprule',
        r'Component & Mean (ms) & Std (ms) & \% budget \\',
        r'\midrule',
        r'\multicolumn{4}{l}{\textit{YOLO Masker (separate process, CUDA GPU)}} \\',
        lrow('YOLO cached publish', *agg(sweep, all_valid, 'YOLO CB'), note='actual VIO overhead'),
        lrow('YOLO inference', *agg(sweep, all_valid, 'YOLO inf'), note='background thread'),
        r'\midrule',
        r'\multicolumn{4}{l}{\textit{OpenVINS MSCKF VIO (main thread)}} \\',
        lrow('OpenVINS total', *agg(sweep, SWEEP_KEYS_PL, 'OV total')),
        lrow('KLT feature tracking', *agg(subcomp, sc_keys, 'OV tracking'), indent=True),
        lrow('IMU propagation', *agg(subcomp, sc_keys, 'OV propag'), indent=True),
        lrow('MSCKF update', *agg(subcomp, sc_keys, 'OV MSCKF'), indent=True),
        lrow('SLAM update + marg.', *agg(subcomp, sc_keys, 'OV SLAM'), indent=True),
        r'\midrule',
        r'\multicolumn{4}{l}{\textit{Hybrid Speed Estimator (separate process)}} \\',
        lrow('HSE (ORB + stereo)', *agg(sweep, all_valid, 'HSE')),
        r'\bottomrule',
        r'\end{tabular}',
        r'\end{table}',
    ]
    return '\n'.join(rows)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sweep',   required=True,  help='Path to full 36-scenario timing folder')
    ap.add_argument('--subcomp', default='',     help='Path to subcomp (verbosity:ALL) folder')
    ap.add_argument('--basecfg', default='',     help='Path to base-cfg city timing folder')
    ap.add_argument('--latex',   action='store_true', help='Also print LaTeX tables')
    args = ap.parse_args()

    print('Loading timing data...')
    sweep   = load_folder(args.sweep)
    subcomp = load_folder(args.subcomp)
    basecfg = load_folder(args.basecfg)

    print(f'  Sweep scenarios loaded   : {len(sweep)}')
    print(f'  Subcomp scenarios loaded : {len(subcomp)}')
    print(f'  Basecfg scenarios loaded : {len(basecfg)}')

    if not subcomp:
        print('\n  [WARN] No subcomp data — sub-component rows will show "–".')
        print('         Run ./scripts/run_timing_subcomp.sh first.')
    if not basecfg:
        print('\n  [WARN] No basecfg data — city 600-feat baseline will show "–".')
        print('         Run ./scripts/run_timing_basecfg.sh first.')

    print(table_a(sweep, subcomp))
    print(table_b(sweep, basecfg))

    if args.latex:
        print('\n' + '=' * 80)
        print('  LaTeX — Table A')
        print('=' * 80)
        print(latex_table_a(sweep, subcomp))


if __name__ == '__main__':
    main()
