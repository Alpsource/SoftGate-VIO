import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from scipy.interpolate import interp1d

# ==========================================
# CONFIGURATION
# ==========================================
BASE_DIR = os.getcwd()
FOLDERS = {
    # Parking lot
    "Unmasked PL None":  "unmasked-parking_lot-none",
    "Unmasked PL Low":   "unmasked-parking_lot-low",
    "Unmasked PL Mid":   "unmasked-parking_lot-mid",
    "Unmasked PL High":  "unmasked-parking_lot-high",
    "Masked PL None":    "masked-parking_lot-none",
    "Masked PL Low":     "masked-parking_lot-low",
    "Masked PL Mid":     "masked-parking_lot-mid",
    "Masked PL High":    "masked-parking_lot-high",
    # City day
    "Unmasked CD None":  "unmasked-city_day-none",
    "Unmasked CD Low":   "unmasked-city_day-low",
    "Unmasked CD Mid":   "unmasked-city_day-mid",
    "Unmasked CD High":  "unmasked-city_day-high",
    "Masked CD None":    "masked-city_day-none",
    "Masked CD Low":     "masked-city_day-low",
    "Masked CD Mid":     "masked-city_day-mid",
    "Masked CD High":    "masked-city_day-high",
    # City night
    "Unmasked CN None":  "unmasked-city_night-none",
    "Unmasked CN Low":   "unmasked-city_night-low",
    "Unmasked CN Mid":   "unmasked-city_night-mid",
    "Unmasked CN High":  "unmasked-city_night-high",
    "Masked CN None":    "masked-city_night-none",
    "Masked CN Low":     "masked-city_night-low",
    "Masked CN Mid":     "masked-city_night-mid",
    "Masked CN High":    "masked-city_night-high",
}

ENABLE_SCALE_ALIGNMENT = False  
FAILURE_THRESHOLD_RMSE = 10.0   

# ==========================================
# ALIGNMENT ENGINE (The Math Part)
# ==========================================
def align_trajectory(gt_data, eval_data, is_rtabmap=False):
    gt_t = gt_data['timestamp'].to_numpy()
    gt_xyz = gt_data[['x', 'y', 'z']].to_numpy()
    
    eval_t = eval_data['timestamp'].to_numpy()
    eval_xyz = eval_data[['x', 'y', 'z']].to_numpy()

    if is_rtabmap:
        time_offset = gt_t[0] - eval_t[0]
        eval_t = eval_t + time_offset

    t_start = max(gt_t[0], eval_t[0])
    t_end = min(gt_t[-1], eval_t[-1])
    
    mask = (eval_t >= t_start) & (eval_t <= t_end)
    eval_t_sync = eval_t[mask]
    eval_xyz_sync = eval_xyz[mask]
    
    if len(eval_t_sync) < 5: 
        return None, 9999.0, 1.0 

    interp_func = interp1d(gt_t, gt_xyz, axis=0, kind='linear')
    gt_xyz_sync = interp_func(eval_t_sync)

    mu_gt = np.mean(gt_xyz_sync, axis=0)
    mu_eval = np.mean(eval_xyz_sync, axis=0)

    gt_centered = gt_xyz_sync - mu_gt
    eval_centered = eval_xyz_sync - mu_eval

    scale = 1.0
    if ENABLE_SCALE_ALIGNMENT:
        var_gt = np.sum(gt_centered**2) / len(gt_centered)
        var_eval = np.sum(eval_centered**2) / len(eval_centered)
        scale = np.sqrt(var_gt / var_eval)

    H = np.dot(eval_centered.T, gt_centered)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = mu_gt - scale * np.dot(R, mu_eval)

    eval_xyz_aligned = (scale * np.dot(R, eval_xyz.T)).T + t
    eval_sync_aligned = (scale * np.dot(R, eval_xyz_sync.T)).T + t
    
    error = gt_xyz_sync - eval_sync_aligned
    rmse = np.sqrt(np.mean(np.sum(error**2, axis=1)))

    return eval_xyz_aligned, rmse, scale

def path_length_error(gt_data, eval_data):
    """
    Computes Relative Path Length Error (RPE-length):
        |path_eval - path_gt| / path_gt
    Uses raw (non-aligned) trajectories. Synchronized by timestamp.
    This is the metric that captures scale drift — SE(3) alignment absorbs it.
    """
    gt_t   = gt_data['timestamp'].to_numpy()
    eval_t = eval_data['timestamp'].to_numpy()

    t_start = max(gt_t[0], eval_t[0])
    t_end   = min(gt_t[-1], eval_t[-1])

    gt_mask   = (gt_t   >= t_start) & (gt_t   <= t_end)
    eval_mask = (eval_t >= t_start) & (eval_t <= t_end)

    gt_pts   = gt_data[['x','y','z']].to_numpy()[gt_mask]
    eval_pts = eval_data[['x','y','z']].to_numpy()[eval_mask]

    if len(gt_pts) < 2 or len(eval_pts) < 2:
        return 9999.0

    gt_len   = float(np.linalg.norm(np.diff(gt_pts,   axis=0), axis=1).sum())
    eval_len = float(np.linalg.norm(np.diff(eval_pts, axis=0), axis=1).sum())

    if gt_len < 1e-6:
        return 9999.0

    return abs(eval_len - gt_len) / gt_len  # dimensionless ratio

# ==========================================
# SUBPLOT ANALYSIS LOGIC
# ==========================================
def analyze_and_plot(name, level, folder_path, ax):
    vio_files = glob.glob(os.path.join(folder_path, "vio_path_run_*.csv"))
    run_ids = sorted([int(f.split('_')[-1].split('.')[0]) for f in vio_files])

    ax.set_title(name, fontsize=14, fontweight='bold')
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_aspect('equal', 'datalim') 

    results = []
    gt_plotted = False
    
    # --- 1. SAF RTAB-MAP (Turuncu) ---
    pure_rmse = 9999.0
    pure_csv_name = f"rtabmap_path_run_pure_{level.lower()}.csv"
    pure_path = os.path.join(BASE_DIR, pure_csv_name)
    if not os.path.exists(pure_path):
        pure_path = os.path.join(BASE_DIR, "sim_results", pure_csv_name)

    if os.path.exists(pure_path):
        pure_df = pd.read_csv(pure_path)
        gt_file = os.path.join(folder_path, "gt_path_run_1.csv") 
        if os.path.exists(gt_file) and len(pure_df) > 5:
            gt_df_pure = pd.read_csv(gt_file)
            aligned_pure, pure_rmse, _ = align_trajectory(gt_df_pure, pure_df, is_rtabmap=True)
            if aligned_pure is not None and pure_rmse < FAILURE_THRESHOLD_RMSE:
                ax.plot(aligned_pure[:, 0], aligned_pure[:, 1], color='orange', alpha=0.9, linewidth=2, linestyle='-.', zorder=3)

    if not run_ids:
        ax.text(0.5, 0.5, 'Veri Bulunamadı', horizontalalignment='center', verticalalignment='center', transform=ax.transAxes)
        return []

    for rid in run_ids:
        vio_path = os.path.join(folder_path, f"vio_path_run_{rid}.csv")
        fused_path = os.path.join(folder_path, f"fused_path_run_{rid}.csv") 
        gtsam_path = os.path.join(folder_path, f"gtsam_path_run_{rid}.csv") # YENİ: MOT-SLAM Verisi
        gt_path = os.path.join(folder_path, f"gt_path_run_{rid}.csv")

        if not os.path.exists(gt_path): continue

        gt_df = pd.read_csv(gt_path)
        
        if not gt_plotted:
            ax.plot(gt_df['x'], gt_df['y'], 'k--', linewidth=2, label='Ground Truth', zorder=6)
            ax.scatter(gt_df.iloc[0]['x'], gt_df.iloc[0]['y'], c='black', marker='s', s=40, zorder=7)
            gt_plotted = True

        # --- 2. SAF OPENVINS (Yeşil) ---
        vio_rmse = 9999.0
        if os.path.exists(vio_path):
            vio_df = pd.read_csv(vio_path)
            vio_ple = path_length_error(gt_df, vio_df) if os.path.exists(vio_path) else 9999.0
            aligned_vio_xyz, vio_rmse, _ = align_trajectory(gt_df, vio_df, is_rtabmap=False)
            if aligned_vio_xyz is not None and vio_rmse <= FAILURE_THRESHOLD_RMSE:
                color = 'green' if name.lower().startswith('unmasked') else 'magenta'
                ax.plot(aligned_vio_xyz[:, 0], aligned_vio_xyz[:, 1], color=color, alpha=0.3, linewidth=1, zorder=2)

        # --- 3. GTSAM MOT-SLAM (Mor - Yeni!) ---
        gtsam_rmse = 9999.0
        if os.path.exists(gtsam_path):
            gtsam_df = pd.read_csv(gtsam_path)
            if len(gtsam_df) > 5: 
                aligned_gtsam_xyz, gtsam_rmse, _ = align_trajectory(gt_df, gtsam_df, is_rtabmap=False)
                if aligned_gtsam_xyz is not None and gtsam_rmse <= FAILURE_THRESHOLD_RMSE:
                    ax.plot(aligned_gtsam_xyz[:, 0], aligned_gtsam_xyz[:, 1], color='purple', alpha=0.8, linewidth=2, zorder=4)

        # --- DOV-SLAM (Cyan - New contribution) ---
        dov_path = os.path.join(folder_path, f"dov_path_run_{rid}.csv")
        dov_rmse = 9999.0
        dov_ple = 9999.0
        if os.path.exists(dov_path):
            dov_df = pd.read_csv(dov_path)
            dov_ple = path_length_error(gt_df, dov_df) if len(dov_df) > 5 else 9999.0
            if len(dov_df) > 5:
                aligned_dov_xyz, dov_rmse, _ = align_trajectory(gt_df, dov_df, is_rtabmap=False)
                if aligned_dov_xyz is not None and dov_rmse <= FAILURE_THRESHOLD_RMSE:
                    ax.plot(aligned_dov_xyz[:, 0], aligned_dov_xyz[:, 1],
                            color='cyan', alpha=0.9, linewidth=2.5, zorder=7)
                    
        # --- 4. NİHAİ FÜZYON (Mavi) ---
        fused_rmse = 9999.0
        if os.path.exists(fused_path):
            fused_df = pd.read_csv(fused_path)
            if len(fused_df) > 5: 
                aligned_fused_xyz, fused_rmse, _ = align_trajectory(gt_df, fused_df, is_rtabmap=False)
                if aligned_fused_xyz is not None and fused_rmse <= FAILURE_THRESHOLD_RMSE:
                    ax.plot(aligned_fused_xyz[:, 0], aligned_fused_xyz[:, 1], color='blue', alpha=0.6, linewidth=2, zorder=5)

        # primary_rmse = min(r for r in [vio_rmse, m_vio_rmse if 'Masked' in name else 9999.0] if r < 9999.0)
        status = "GOOD" if vio_rmse <= FAILURE_THRESHOLD_RMSE else "BAD"
        results.append({
            'run': rid, 
            'vio_rmse': vio_rmse, 
            'dov_rmse': dov_rmse,
            'vio_ple':  vio_ple,
            'dov_ple':  dov_ple,
            'status': status
        })

    return results

# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--median", action="store_true",
                        help="Report median instead of mean over valid runs")
    parser.add_argument("--env", type=str, default="parking_lot",
                        choices=["parking_lot", "city_day", "city_night"],
                        help="Which environment to analyse")
    parser.add_argument("--compare-prefix", type=str, default="masked",
                        help="Folder prefix for comparison row (default: masked). "
                             "Use 'ekf' to compare ekf-* results vs unmasked baseline.")
    args = parser.parse_args()
    ENV = args.env
    USE_MEDIAN = args.median
    COMPARE_PREFIX = args.compare_prefix

    # Build the 8-panel layout: rows = Unmasked/Compare, cols = None/Low/Mid/High
    DENSITIES = ["none", "low", "mid", "high"]
    MODES     = ["unmasked", COMPARE_PREFIX]

    cmp_label = COMPARE_PREFIX.replace("_", " ").capitalize()
    fig, axes = plt.subplots(2, 4, figsize=(28, 13))
    fig.suptitle(
        f"VIO Trajectory Analysis — {ENV.replace('_',' ').title()}\n"
        f"Green=Unmasked VIO   Magenta={cmp_label} VIO   Cyan=DOV-corrected   Black=Ground Truth",
        fontsize=14, fontweight='bold'
    )

    all_results = {}

    for row, mode in enumerate(MODES):
        for col, density in enumerate(DENSITIES):
            folder_name = f"{mode}-{ENV}-{density}"
            label       = f"{mode.capitalize()} — {density.capitalize()}"
            ax          = axes[row, col]
            full_path   = os.path.join(BASE_DIR, folder_name)

            if os.path.exists(full_path):
                all_results[label] = analyze_and_plot(label, density, full_path, ax)
            else:
                ax.set_title(label)
                ax.text(0.5, 0.5, 'Not found', ha='center', va='center',
                        transform=ax.transAxes)
                all_results[label] = []

    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='k',       lw=2, linestyle='--', label='Ground Truth'),
        Line2D([0], [0], color='green',   lw=1, alpha=0.5,      label='Unmasked VIO'),
        Line2D([0], [0], color='magenta', lw=1, alpha=0.5,      label=f'{cmp_label} VIO'),
        Line2D([0], [0], color='cyan',    lw=2, alpha=0.9,      label='DOV-corrected'),
    ]
    axes[0, 0].legend(handles=legend_elements, loc='best', fontsize=9)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    out_path = os.path.join(BASE_DIR, f"Trajectory_Analysis_{ENV}.png")
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"\n[INFO] Plot saved: {out_path}")

    from scipy.stats import wilcoxon

    print("\n" + "="*78)
    print(f"RESULTS SUMMARY — {ENV.replace('_',' ').upper()}")
    print(f"Metric: ATE (SE3-aligned RMSE, m)  |  divergence threshold: {FAILURE_THRESHOLD_RMSE} m")
    print(f"Aggregation: mean ± std over valid runs  |  Wilcoxon: one-tailed (masking helps)")
    print("="*78)

    def get_stats(res_list, key, threshold=FAILURE_THRESHOLD_RMSE):
        valid = [r[key] for r in res_list if r[key] < threshold]
        if not valid:
            return float('nan'), float('nan'), 0, len(res_list)
        return float(np.mean(valid)), float(np.std(valid, ddof=1) if len(valid) > 1 else 0.0), len(valid), len(res_list)

    def get_ple_stats(res_list, key):
        valid = [r[key] for r in res_list if r[key] < 9.0]
        if not valid:
            return float('nan'), 0
        return float(np.mean(valid)), len(valid)

    def paired_wilcoxon(u_res, m_res, key, threshold=FAILURE_THRESHOLD_RMSE):
        u_by_run = {r['run']: r[key] for r in u_res if r[key] < threshold}
        m_by_run = {r['run']: r[key] for r in m_res if r[key] < threshold}
        common = sorted(set(u_by_run) & set(m_by_run))
        if len(common) < 4:
            return float('nan'), len(common)
        diffs = [u_by_run[rid] - m_by_run[rid] for rid in common]
        if all(d == 0 for d in diffs):
            return float('nan'), len(common)
        try:
            _, p = wilcoxon(diffs, alternative='greater')
        except Exception:
            return float('nan'), len(common)
        return p, len(common)

    def fmt(v):    return f"{v:.3f}" if not np.isnan(v) else "  N/A "
    def fmts(v):   return f"{v:.3f}" if not np.isnan(v) else "N/A"
    def fmtp(v):   return f"{v*100:.1f}%" if not np.isnan(v) else " N/A "
    def fmtpv(v):  return f"{v:.3f}" if not np.isnan(v) else "  N/A"

    cmp_short = COMPARE_PREFIX[:3].upper()

    # ── VIO ATE table ────────────────────────────────────────────────────────
    print(f"\n{'Density':<8}  {'U-ATE mean±std':>17}  {'U N':>5}  "
          f"{'M-ATE mean±std':>17}  {'M N':>5}  {'Mask Δ':>7}  {'p-value':>8}")
    print("-" * 78)

    for density in DENSITIES:
        u_label = f"Unmasked — {density.capitalize()}"
        m_label = f"{cmp_label} — {density.capitalize()}"
        u_res   = all_results.get(u_label, [])
        m_res   = all_results.get(m_label, [])

        u_mean, u_std, u_n, u_total = get_stats(u_res, 'vio_rmse')
        m_mean, m_std, m_n, m_total = get_stats(m_res, 'vio_rmse')
        p_val, n_pairs = paired_wilcoxon(u_res, m_res, 'vio_rmse')

        if not np.isnan(u_mean) and not np.isnan(m_mean) and u_mean > 1e-6:
            mask_delta = f"{(u_mean - m_mean) / u_mean * 100:+.1f}%"
        else:
            mask_delta = "  N/A "

        p_str = f"{p_val:.3f}" if not np.isnan(p_val) else "  N/A"
        if not np.isnan(p_val):
            p_str += "*" if p_val < 0.05 else " "

        u_cell = f"{fmt(u_mean)}±{fmts(u_std)}"
        m_cell = f"{fmt(m_mean)}±{fmts(m_std)}"
        print(f"{density:<8}  {u_cell:>17}  {u_n}/{u_total:>3}  "
              f"{m_cell:>17}  {m_n}/{m_total:>3}  {mask_delta:>7}  {p_str:>8}")

    # ── DOV ATE table ─────────────────────────────────────────────────────────
    print(f"\n{'Density':<8}  {'M-VIO ATE':>10}  {'M-DOV ATE':>10}  {'DOV Δ':>7}  {'p-value':>8}")
    print("-" * 52)

    for density in DENSITIES:
        u_label = f"Unmasked — {density.capitalize()}"
        m_label = f"{cmp_label} — {density.capitalize()}"
        u_res   = all_results.get(u_label, [])
        m_res   = all_results.get(m_label, [])

        m_vio_mean, _, m_vio_n, m_total = get_stats(m_res, 'vio_rmse')
        m_dov_mean, _, m_dov_n, _       = get_stats(m_res, 'dov_rmse')
        p_dov, _ = paired_wilcoxon(m_res, m_res, 'dov_rmse')  # placeholder structure

        # Correct DOV Wilcoxon: vio_rmse vs dov_rmse within masked condition
        m_by_run_vio = {r['run']: r['vio_rmse'] for r in m_res if r['vio_rmse'] < FAILURE_THRESHOLD_RMSE}
        m_by_run_dov = {r['run']: r['dov_rmse'] for r in m_res if r['dov_rmse'] < FAILURE_THRESHOLD_RMSE}
        dov_common = sorted(set(m_by_run_vio) & set(m_by_run_dov))
        if len(dov_common) >= 4:
            dov_diffs = [m_by_run_vio[rid] - m_by_run_dov[rid] for rid in dov_common]
            try:
                _, p_dov = wilcoxon(dov_diffs, alternative='greater') if not all(d == 0 for d in dov_diffs) else (None, float('nan'))
            except Exception:
                p_dov = float('nan')
        else:
            p_dov = float('nan')

        if not np.isnan(m_vio_mean) and not np.isnan(m_dov_mean) and m_vio_mean > 1e-6:
            dov_delta = f"{(m_vio_mean - m_dov_mean) / m_vio_mean * 100:+.1f}%"
        else:
            dov_delta = "  N/A "

        p_dov_str = f"{p_dov:.3f}" if not np.isnan(p_dov) else "  N/A"
        if not np.isnan(p_dov):
            p_dov_str += "*" if p_dov < 0.05 else " "

        print(f"{density:<8}  {fmt(m_vio_mean):>10}  {fmt(m_dov_mean):>10}  "
              f"{dov_delta:>7}  {p_dov_str:>8}")

    # ── PLE table ─────────────────────────────────────────────────────────────
    print(f"\n{'Density':<8}  {'U-VIO PLE':>10}  {f'{cmp_short}-VIO PLE':>10}  "
          f"{'U-DOV PLE':>10}  {f'{cmp_short}-DOV PLE':>10}")
    print("-" * 56)

    for density in DENSITIES:
        u_label = f"Unmasked — {density.capitalize()}"
        m_label = f"{cmp_label} — {density.capitalize()}"
        u_res   = all_results.get(u_label, [])
        m_res   = all_results.get(m_label, [])

        u_vio_ple, _ = get_ple_stats(u_res, 'vio_ple')
        m_vio_ple, _ = get_ple_stats(m_res, 'vio_ple')
        u_dov_ple, _ = get_ple_stats(u_res, 'dov_ple')
        m_dov_ple, _ = get_ple_stats(m_res, 'dov_ple')

        print(f"{density:<8}  {fmtp(u_vio_ple):>10}  {fmtp(m_vio_ple):>10}  "
              f"{fmtp(u_dov_ple):>10}  {fmtp(m_dov_ple):>10}")

    print(f"\nU=Unmasked  {cmp_short}=Masked  ATE in m (lower=better)  PLE=path length error %")
    print(f"Mask Δ: (U-ATE − M-ATE)/U-ATE  |  DOV Δ: (VIO-ATE − DOV-ATE)/VIO-ATE")
    print(f"Wilcoxon: one-tailed, paired on run IDs valid in both conditions  |  *p<0.05")