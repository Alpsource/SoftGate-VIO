#!/usr/bin/env python3
"""
analyze_ablation_batches.py
---------------------------
Reads all ablation_batches/batch_N/ directories, parses analysis.log and
params.txt, and prints a structured side-by-side comparison.

Run from sim_results/:
    python3 analyze_ablation_batches.py
or with a custom batch root:
    python3 analyze_ablation_batches.py --batch_root /path/to/ablation_batches
"""

import os
import re
import sys
import argparse
import glob

# ── CONFIG ────────────────────────────────────────────────────────────────────
DEFAULT_BATCH_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "ablation_batches")
ENVS      = ["parking_lot", "city_day", "city_night"]
DENSITIES = ["none", "low", "mid", "high"]

ENV_HEADER_MAP = {
    "PARKING LOT": "parking_lot",
    "CITY DAY":    "city_day",
    "CITY NIGHT":  "city_night",
}

# ── PARSING ───────────────────────────────────────────────────────────────────

def parse_params(params_path):
    params = {}
    if not os.path.isfile(params_path):
        return params
    with open(params_path) as f:
        for line in f:
            if '=' in line:
                k, v = line.split('=', 1)
                params[k.strip()] = v.strip()
    return params


def parse_analysis_log(log_path):
    """
    Returns dict: env -> density -> {u_vio_ate, m_vio_ate, m_dov_ate,
                                      mask_delta, dov_delta}
    Missing data is stored as None.
    """
    results = {env: {d: None for d in DENSITIES} for env in ENVS}
    current_env = None
    in_table    = False

    if not os.path.isfile(log_path):
        return results

    with open(log_path, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    for line in lines:
        # Detect environment section
        for header, env_key in ENV_HEADER_MAP.items():
            if f"RESULTS SUMMARY — {header}" in line:
                current_env = env_key
                in_table    = False
                break

        # Detect table header row
        if current_env and "Density" in line and "U-VIO ATE" in line:
            in_table = True
            continue

        # Separator ends the table
        if in_table and re.match(r'\s*-{10,}', line):
            continue   # skip separator, don't end the table yet

        if in_table and re.match(r'\s*Columns:', line):
            in_table = False
            continue

        # Parse data row
        if in_table and current_env:
            parts = line.split()
            if len(parts) >= 11 and parts[0] in DENSITIES:
                density = parts[0]
                try:
                    u_vio = float(parts[1])
                    m_vio = float(parts[2])
                    m_dov = float(parts[4])
                    mask_d = float(parts[9].replace('%','').replace('+',''))
                    dov_d  = float(parts[10].replace('%','').replace('+',''))
                    results[current_env][density] = {
                        'u_vio_ate':  u_vio,
                        'm_vio_ate':  m_vio,
                        'm_dov_ate':  m_dov,
                        'mask_delta': mask_d,
                        'dov_delta':  dov_d,
                    }
                except (ValueError, IndexError):
                    pass

    return results


def load_all_batches(batch_root, log_filename="analysis.log"):
    batch_dirs = sorted(glob.glob(os.path.join(batch_root, "batch_*")),
                        key=lambda p: int(re.search(r'batch_(\d+)', p).group(1)))
    batches = []
    for bd in batch_dirs:
        num_match = re.search(r'batch_(\d+)', bd)
        if not num_match:
            continue
        num     = int(num_match.group(1))
        params  = parse_params(os.path.join(bd, "params.txt"))
        results = parse_analysis_log(os.path.join(bd, log_filename))
        batches.append({
            'num':     num,
            'dir':     bd,
            'params':  params,
            'results': results,
        })
    return batches

# ── SCORING ───────────────────────────────────────────────────────────────────

def dov_score(results, env="parking_lot"):
    """Sum of DOV Δ for low/mid/high (penalise negatives ×2)."""
    s = 0.0
    for d in ["low", "mid", "high"]:
        row = results.get(env, {}).get(d)
        if row:
            v = row['dov_delta']
            s += v if v >= 0 else 2 * v
    return s


def vio_score(results):
    """Sum of Mask Δ for low/mid/high across all environments (penalise ×2)."""
    s = 0.0
    for env in ENVS:
        for d in ["low", "mid", "high"]:
            row = results.get(env, {}).get(d)
            if row:
                v = row['mask_delta']
                s += v if v >= 0 else 2 * v
    return s


def combined_score(results):
    """DOV score (parking_lot) + 0.3 × VIO score (all envs)."""
    return dov_score(results) + 0.3 * vio_score(results)

# ── FORMATTING HELPERS ────────────────────────────────────────────────────────

def fmt_pct(val, width=7):
    if val is None:
        return f"{'N/A':>{width}}"
    sign = '+' if val >= 0 else ''
    return f"{sign}{val:.1f}%".rjust(width)


def fmt_ate(val, width=7):
    if val is None:
        return f"{'N/A':>{width}}"
    return f"{val:.3f}m".rjust(width)


def batch_label(b):
    return b['params'].get('label', f"batch_{b['num']}")


def changed_param(b):
    """Return the parameter name that differs from defaults."""
    defaults = dict(dilation_kernel='13', max_mask_fraction='0.80',
                    min_disparity='1.2', min_features='8', orb_nfeatures='200')
    changed = []
    for k, default_v in defaults.items():
        v = b['params'].get(k, default_v)
        if v != default_v:
            changed.append(f"{k}={v}")
    return ', '.join(changed) if changed else 'all defaults'

# ── PRINT HELPERS ─────────────────────────────────────────────────────────────

def section(title):
    w = 78
    print()
    print('=' * w)
    print(f"  {title}")
    print('=' * w)


def subsection(title):
    print()
    print(f"  ── {title}")
    print()


def mark_best(values, higher_is_better=True):
    """Return list of booleans marking the best value."""
    valid = [v for v in values if v is not None]
    if not valid:
        return [False] * len(values)
    best = max(valid) if higher_is_better else min(valid)
    return [v == best if v is not None else False for v in values]

# ── MAIN DISPLAY ──────────────────────────────────────────────────────────────

def print_params_table(batches):
    section("BATCH PARAMETERS")
    hdr = f"{'#':<4}  {'Label':<16}  {'dilation':>8}  {'mask_frac':>9}  {'min_disp':>8}  {'min_feat':>8}  {'orb':>5}"
    print(hdr)
    print('-' * len(hdr))
    for b in batches:
        p = b['params']
        print(f"  {b['num']:<3}  {batch_label(b):<16}  "
              f"{p.get('dilation_kernel','?'):>8}  "
              f"{p.get('max_mask_fraction','?'):>9}  "
              f"{p.get('min_disparity','?'):>8}  "
              f"{p.get('min_features','?'):>8}  "
              f"{p.get('orb_nfeatures','?'):>5}")


def print_delta_table(batches, metric_key, env, higher_is_better, section_note=""):
    label_w = 16
    col_w   = 8

    hdr_parts = [f"{'#':<3}", f"{'Label':<{label_w}}"]
    for d in DENSITIES:
        hdr_parts.append(f"{d:>{col_w}}")
    hdr_parts.append(f"{'Score':>{col_w}}")
    hdr = "  " + "  ".join(hdr_parts)
    print(hdr)
    print('  ' + '-' * (len(hdr) - 2))

    # Collect values for best-marking
    col_vals = {d: [] for d in DENSITIES}
    scores   = []
    for b in batches:
        for d in DENSITIES:
            row = b['results'].get(env, {}).get(d)
            col_vals[d].append(row[metric_key] if row else None)
        if metric_key == 'dov_delta':
            scores.append(dov_score(b['results'], env))
        else:
            scores.append(sum(
                (b['results'].get(env, {}).get(d) or {}).get(metric_key, 0.0)
                for d in ["low", "mid", "high"]
            ))

    best_cols  = {d: mark_best(col_vals[d], higher_is_better) for d in DENSITIES}
    best_score = mark_best(scores, higher_is_better=True)

    for i, b in enumerate(batches):
        parts = [f"{b['num']:<3}", f"{batch_label(b):<{label_w}}"]
        for d in DENSITIES:
            row = b['results'].get(env, {}).get(d)
            val = row[metric_key] if row else None
            s   = fmt_pct(val, col_w) if metric_key != 'm_dov_ate' else fmt_ate(val, col_w)
            star = '★' if best_cols[d][i] else ' '
            parts.append(f"{s}{star}")
        sc_str = f"{scores[i]:+.1f}"
        star = '★' if best_score[i] else ' '
        parts.append(f"{sc_str:>{col_w}}{star}")
        print("  " + "  ".join(parts))

    if section_note:
        print(f"\n  Note: {section_note}")


def print_ate_table(batches, env, metric_key='m_dov_ate'):
    label_w = 16
    col_w   = 9

    hdr_parts = [f"{'#':<3}", f"{'Label':<{label_w}}"]
    for d in DENSITIES:
        hdr_parts.append(f"{d:>{col_w}}")
    hdr = "  " + "  ".join(hdr_parts)
    print(hdr)
    print('  ' + '-' * (len(hdr) - 2))

    col_vals = {d: [] for d in DENSITIES}
    for b in batches:
        for d in DENSITIES:
            row = b['results'].get(env, {}).get(d)
            col_vals[d].append(row[metric_key] if row else None)

    best_cols = {d: mark_best(col_vals[d], higher_is_better=False) for d in DENSITIES}

    for i, b in enumerate(batches):
        parts = [f"{b['num']:<3}", f"{batch_label(b):<{label_w}}"]
        for d in DENSITIES:
            row  = b['results'].get(env, {}).get(d)
            val  = row[metric_key] if row else None
            s    = fmt_ate(val, col_w)
            star = '★' if best_cols[d][i] else ' '
            parts.append(f"{s}{star}")
        print("  " + "  ".join(parts))


def print_ranking(batches):
    section("RANKING SUMMARY")

    # DOV ranking (parking_lot)
    dov_ranked = sorted(batches, key=lambda b: dov_score(b['results']), reverse=True)
    subsection("By DOV Score — parking_lot (DOV Δ for low+mid+high, negatives penalised ×2)")
    for rank, b in enumerate(dov_ranked, 1):
        sc = dov_score(b['results'])
        changed = changed_param(b)
        pl_vals = []
        for d in ['low','mid','high']:
            row = b['results'].get('parking_lot',{}).get(d)
            pl_vals.append(fmt_pct(row['dov_delta'] if row else None, 6))
        print(f"  {rank:>2}. batch_{b['num']:>2}  [{batch_label(b):<16}]  "
              f"score={sc:+6.2f}   low={pl_vals[0]}  mid={pl_vals[1]}  high={pl_vals[2]}")

    # VIO ranking (all envs)
    vio_ranked = sorted(batches, key=lambda b: vio_score(b['results']), reverse=True)
    subsection("By VIO Score — all envs (Mask Δ for low+mid+high, negatives penalised ×2)")
    for rank, b in enumerate(vio_ranked, 1):
        sc = vio_score(b['results'])
        changed = changed_param(b)
        # Show per-env masked ATE at high density
        vals = []
        for env in ENVS:
            row = b['results'].get(env, {}).get('high')
            vals.append(fmt_ate(row['m_vio_ate'] if row else None, 7))
        print(f"  {rank:>2}. batch_{b['num']:>2}  [{batch_label(b):<16}]  "
              f"score={sc:+7.1f}   PL-high={vals[0]}  CD-high={vals[1]}  CN-high={vals[2]}")

    # Combined
    combined_ranked = sorted(batches, key=lambda b: combined_score(b['results']), reverse=True)
    subsection("By Combined Score (DOV + 0.3 × VIO)")
    for rank, b in enumerate(combined_ranked, 1):
        sc = combined_score(b['results'])
        print(f"  {rank:>2}. batch_{b['num']:>2}  [{batch_label(b):<16}]  score={sc:+7.2f}")

    # Best recommendation
    best = combined_ranked[0]
    p    = best['params']
    section("RECOMMENDED CONFIGURATION  (highest combined score)")
    print(f"  Batch     : batch_{best['num']}  [{batch_label(best)}]")
    print(f"  Changed   : {changed_param(best)}")
    print()
    print(f"  dilation_kernel   = {p.get('dilation_kernel','?')}")
    print(f"  max_mask_fraction = {p.get('max_mask_fraction','?')}")
    print(f"  min_disparity     = {p.get('min_disparity','?')}")
    print(f"  min_features      = {p.get('min_features','?')}")
    print(f"  orb_nfeatures     = {p.get('orb_nfeatures','?')}")
    print()
    print("  Scores:")
    print(f"    DOV score (parking_lot) = {dov_score(best['results']):+.2f}")
    print(f"    VIO score (all envs)    = {vio_score(best['results']):+.1f}")
    print(f"    Combined                = {combined_score(best['results']):+.2f}")
    print()
    print("  parking_lot results:")
    for d in DENSITIES:
        row = best['results'].get('parking_lot',{}).get(d)
        if row:
            print(f"    {d:<5}  M-VIO={row['m_vio_ate']:.3f}m  "
                  f"M-DOV={row['m_dov_ate']:.3f}m  "
                  f"Mask Δ={row['mask_delta']:+.1f}%  "
                  f"DOV Δ={row['dov_delta']:+.1f}%")


def main():
    parser = argparse.ArgumentParser(description="Ablation batch comparison tool")
    parser.add_argument("--batch_root", default=DEFAULT_BATCH_ROOT,
                        help="Path to ablation_batches/ directory")
    parser.add_argument("--log", default="analysis.log",
                        help="Log filename to read from each batch_N/ (default: analysis.log, "
                             "use analysis_consensus.log for consensus results)")
    args = parser.parse_args()

    batches = load_all_batches(args.batch_root, log_filename=args.log)
    if not batches:
        print(f"[ERROR] No batch_N/ directories found in: {args.batch_root}")
        sys.exit(1)

    print()
    print("=" * 78)
    print("  VIODE-VIO ABLATION BATCH ANALYSIS")
    print(f"  {len(batches)} batches found  |  {args.batch_root}")
    print("=" * 78)

    print_params_table(batches)

    # ── Section 1: Masking effectiveness ────────────────────────────────────
    section("SECTION 1 — VIO MASKING EFFECTIVENESS  (Mask Δ, higher = better)")
    print("  How much does masking improve VIO vs unmasked? Affected by dilation & mask_frac.")
    for env in ENVS:
        subsection(f"Environment: {env}")
        print_delta_table(batches, 'mask_delta', env, higher_is_better=True)

    # ── Section 2: DOV correction effectiveness ──────────────────────────────
    section("SECTION 2 — DOV CORRECTION EFFECTIVENESS  (DOV Δ, higher = better)")
    print("  How much does DOV improve on top of masked VIO? Affected by estimator params.")
    for env in ENVS:
        note = "" if env != "city_day" else "city envs are fundamentally limited (all objects moving)"
        subsection(f"Environment: {env}")
        print_delta_table(batches, 'dov_delta', env, higher_is_better=True,
                          section_note=note)

    # ── Section 3: Final M-DOV ATE ───────────────────────────────────────────
    section("SECTION 3 — FINAL MASKED+DOV ATE  (meters, lower = better)")
    print("  Absolute accuracy after masking + DOV correction.")
    for env in ENVS:
        subsection(f"Environment: {env}")
        print_ate_table(batches, env, 'm_dov_ate')

    # ── Section 4: Unmasked VIO baseline ─────────────────────────────────────
    section("SECTION 4 — UNMASKED VIO ATE  (meters, lower = better)")
    print("  Raw VIO without any masking — sanity check that parameters don't change VIO.")
    for env in ENVS:
        subsection(f"Environment: {env}")
        print_ate_table(batches, env, 'u_vio_ate')

    # ── Ranking & recommendation ─────────────────────────────────────────────
    print_ranking(batches)

    print()
    print("=" * 78)
    print("  END OF ANALYSIS  — send this output to get parameter recommendations")
    print("=" * 78)
    print()


if __name__ == "__main__":
    main()
