"""
dov_postprocessor.py  —  # DOV-SLAM: World-Frame EKF + RANSAC Ego-Motion Residual Corrector
======================================================================
"""

import pandas as pd
import numpy as np
import os
import glob
from scipy.spatial.transform import Rotation
from filterpy.kalman import KalmanFilter, ExtendedKalmanFilter

# ==========================================
# CONFIGURATION
# ==========================================
BASE_DIR = os.getcwd()
FOLDERS = [
    "masked-parking_lot-none",   "masked-parking_lot-low",
    "masked-parking_lot-mid",    "masked-parking_lot-high",
    "unmasked-parking_lot-none", "unmasked-parking_lot-low",
    "unmasked-parking_lot-mid",  "unmasked-parking_lot-high",
    "masked-city_day-none",      "masked-city_day-low",
    "masked-city_day-mid",       "masked-city_day-high",
    "unmasked-city_day-none",    "unmasked-city_day-low",
    "unmasked-city_day-mid",     "unmasked-city_day-high",
    "masked-city_night-none",    "masked-city_night-low",
    "masked-city_night-mid",     "masked-city_night-high",
    "unmasked-city_night-none",  "unmasked-city_night-low",
    "unmasked-city_night-mid",   "unmasked-city_night-high",
]

# EKF Tuning (FilterPy)
EKF_Q_POS        = 0.05
EKF_Q_VEL        = 1.20
EKF_R_OBS        = 0.10

# RANSAC ego-motion fitting
RANSAC_THRESH    = 0.30    # m/s inlier threshold
MIN_OBJ_COUNT    = 1

# Bias correction
USE_OUTLIER_REJECTION = False   # set True to reject objects >2 m/s from consensus
EMA_ALPHA        = 0.05
MIN_OBJ_FULL     = 4       # weight = min(N/4, 1.0)
CORRECTION_TRUST = 0.40
MAX_CORRECTION   = 1.50    # m/s per step

R_IMU_CAM = Rotation.from_quat([0.5, 0.5, 0.5, 0.5]).as_matrix()

# ==========================================
# EKF NON-LINEAR FUNCTIONS
# ==========================================
def measurement_fn(x):
    """ Non-linear measurement function h(x). """
    return x[0:3]

def measurement_jacobian(x):
    """ Jacobian of the measurement function H_j. """
    H = np.zeros((3, 6))
    H[0, 0] = H[1, 1] = H[2, 2] = 1.0
    return H

def state_transition_fn(x, dt):
    """ Non-linear state transition function f(x, u). """
    F = np.eye(6)
    F[0, 3] = F[1, 4] = F[2, 5] = dt
    return F @ x

def quat_to_rot(qx, qy, qz, qw):
    return Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

# ==========================================
# FILTERPY EKF (WORLD FRAME + CAUSAL)
# ==========================================
def track_object_causal(obj_data):
    n_meas = len(obj_data)
    if n_meas < 3: return None

    t = obj_data['timestamp'].values
    
    # DİKKAT: Artık Kamera değil, Dünya (World) koordinatları EKF'ye giriyor!
    z = obj_data[['x_w', 'y_w', 'z_w']].values

    ekf = ExtendedKalmanFilter(dim_x=6, dim_z=3)
    ekf.x = np.array([z[0, 0], z[0, 1], z[0, 2], 0., 0., 0.])
    ekf.P = np.diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])

    states, covs = [], []

    for i in range(n_meas):
        dt = t[i] - t[i-1] if i > 0 else 0.05
        dt = float(np.clip(dt, 0.01, 1.0))
        
        ekf.x = state_transition_fn(ekf.x, dt)
        ekf.F = np.eye(6)
        ekf.F[0, 3] = ekf.F[1, 4] = ekf.F[2, 5] = dt
        ekf.Q = np.diag([EKF_Q_POS**2 * dt]*3 + [EKF_Q_VEL**2 * dt]*3)
        
        ekf.predict()
        
        # Orijinal Z_cam mesafesini gürültü hesabı için kullanıyoruz
        depth = np.clip(obj_data['z_cam'].values[i], 1.0, 60.0) 
        dynamic_R = EKF_R_OBS + (0.005 * depth**2) 
        
        ekf.update(
            z[i], 
            HJacobian=measurement_jacobian, 
            Hx=measurement_fn, 
            R=np.eye(3) * dynamic_R
        )
        
        states.append(ekf.x.copy())
        covs.append(ekf.P.copy())

    states = np.array(states)
    covs = np.array(covs)

    # --- SİHİR BURADA BAŞLIYOR: DÜNYA KOORDİNATLARINI KAMERAYA GERİ ÇEVİRME ---
    sm_cam_x, sm_cam_y, sm_cam_z = [], [], []
    
    vx = obj_data['x'].values
    vy = obj_data['y'].values
    vz = obj_data['z'].values
    vqx = obj_data['qx'].values
    vqy = obj_data['qy'].values
    vqz = obj_data['qz'].values
    vqw = obj_data['qw'].values

    for i in range(n_meas):
        P_w_sm = states[i, 0:3]
        P_vio = np.array([vx[i], vy[i], vz[i]])
        R_wi = quat_to_rot(vqx[i], vqy[i], vqz[i], vqw[i])
        
        R_cam_to_w = R_wi @ R_IMU_CAM
        # Dünya noktasından VIO'yu çıkarıp ters rotasyonla tekrar kameraya alıyoruz
        P_cam_sm = R_cam_to_w.T @ (P_w_sm - P_vio)
        
        sm_cam_x.append(P_cam_sm[0])
        sm_cam_y.append(P_cam_sm[1])
        sm_cam_z.append(P_cam_sm[2])

    sm_cam_pts = np.array([sm_cam_x, sm_cam_y, sm_cam_z]).T

    # Pürüzsüz Kamera pozisyonlarından Apparent Velocity (Görünen Hız) türetme
    sm_vx, sm_vy, sm_vz = np.zeros(n_meas), np.zeros(n_meas), np.zeros(n_meas)
    for i in range(1, n_meas):
      dt = t[i] - t[i-1]
      if dt > 0.001:
          v = (sm_cam_pts[i] - sm_cam_pts[i-1]) / dt                            
          sm_vx[i], sm_vy[i], sm_vz[i] = v[0], v[1], v[2] 
    
    # Simply leave index 0 as zeros — velocity is unknown at first observation
    # (zeros already set by np.zeros above, so just commented out this block entirely)
    #         
    # if n_meas > 1:
    #     sm_vx[0], sm_vy[0], sm_vz[0] = sm_vx[1], sm_vy[1], sm_vz[1]

    # World-frame velocity magnitude from EKF state — used to down-weight dynamic objects
    world_vel_mag = np.linalg.norm(states[:, 3:6], axis=1)

    tracked_df = pd.DataFrame({
        'timestamp': t,
        'sm_x': sm_cam_pts[:, 0], 'sm_y': sm_cam_pts[:, 1], 'sm_z': sm_cam_pts[:, 2],
        'sm_vx': sm_vx, 'sm_vy': sm_vy, 'sm_vz': sm_vz,
        'weight': 1.0 / (np.trace(covs[:, 3:6, 3:6], axis1=1, axis2=2) + 0.01),
        'world_vel_mag': world_vel_mag
    })
    return tracked_df

def build_ego_system(positions, velocities):
    A, b = [], []                                                             
    for P, v in zip(positions, velocities):
        X, Y, Z = P                                                           
        if Z < 0.5: continue                                                  
        A.append([-1.0, 0.0, 0.0])
        b.append(v[0])                                                        
        A.append([0.0, -1.0, 0.0])
        b.append(v[1])                                                        
        A.append([0.0, 0.0, -1.0])
        b.append(v[2])                                                        
    return np.array(A), np.array(b)


# ==========================================
# MAIN DOV PIPELINE
# ==========================================
def compute_dov_correction(vio_df, obj_df):
    n = len(vio_df)
    vio_t = vio_df['timestamp'].values
    vio_xyz = vio_df[['x', 'y', 'z']].values
    
    has_q = all(c in vio_df.columns for c in ['qx','qy','qz','qw'])
    vio_q = vio_df[['qx','qy','qz','qw']].values if has_q else np.tile([0.,0.,0.,1.], (n,1))

    if obj_df.empty: return np.zeros((n,3))

    # --- YENİ ADIM: VIO ile OBJ Verilerini Eşleştir ve Dünya Koordinatlarına Çevir ---
    obj_df_sorted = obj_df.sort_values('timestamp')
    vio_df_sorted = vio_df.sort_values('timestamp')
    
    merged = pd.merge_asof(obj_df_sorted, vio_df_sorted, on='timestamp', direction='backward') # Only past or same-time (backward)
    
    # Drop rows where VIO pose was not found (NaN quaternions = no backward match)
    merged = merged.dropna(subset=['qx', 'qy', 'qz', 'qw', 'x', 'y', 'z']).reset_index(drop=True)
    if merged.empty:
        return np.zeros((n, 3))

    world_pts = []
    for _, row in merged.iterrows():
        R_wi = quat_to_rot(row['qx'], row['qy'], row['qz'], row['qw'])
        P_cam = np.array([row['x_cam'], row['y_cam'], row['z_cam']])
        P_vio = np.array([row['x'], row['y'], row['z']])
        P_w = P_vio + R_wi @ R_IMU_CAM @ P_cam
        world_pts.append(P_w)
        
    world_pts = np.array(world_pts)
    merged['x_w'] = world_pts[:, 0]
    merged['y_w'] = world_pts[:, 1]
    merged['z_w'] = world_pts[:, 2]

    # 1. Batch Process (Artık Dünya koordinatlarında Causal EKF çalışıyor)
    tracked_objects = []
    for oid, group in merged.groupby('obj_id'):
        sm_df = track_object_causal(group)
        if sm_df is not None:
            sm_df['obj_id'] = oid
            tracked_objects.append(sm_df)
            
    if not tracked_objects: return np.zeros((n,3))
    all_smoothed = pd.concat(tracked_objects, ignore_index=True)

    # 2. VIO velocity in world frame
    vio_vel = np.zeros((n, 3))
    for i in range(1, n):
        dt = vio_t[i] - vio_t[i-1]
        if dt > 1e-6: vio_vel[i] = (vio_xyz[i] - vio_xyz[i-1]) / dt
    vio_vel[0] = np.zeros(3)

    raw_corr = np.zeros((n, 3))
    corr_cnt = np.zeros(n, dtype=int)

    # 3. Frame-by-frame correction via velocity-weighted ego estimation
    # Works with even 1 object — static objects (low world vel) dominate, dynamic ones are down-weighted
    for i in range(n):
        t = vio_t[i]

        time_mask = (all_smoothed['timestamp'] >= t - 0.20) & (all_smoothed['timestamp'] <= t)
        active_objs = all_smoothed[time_mask].groupby('obj_id').mean()

        if len(active_objs) < MIN_OBJ_COUNT: continue

        velocities    = active_objs[['sm_vx', 'sm_vy', 'sm_vz']].values
        ekf_weights   = active_objs['weight'].values
        world_vel_mag = active_objs['world_vel_mag'].values
        depths        = np.maximum(active_objs['sm_z'].values, 0.5)

        # Down-weight dynamic objects (static parked cars dominate)
        static_weights = 1.0 / (world_vel_mag + 0.5)

        # Down-weight far objects — stereo depth variance scales as Z²
        depth_weights = 1.0 / (depths ** 2)

        combined_weights = ekf_weights * static_weights * depth_weights
        combined_weights /= combined_weights.sum() + 1e-9

        # Individual ego-velocity estimates from each object: v_ego_i = -v_apparent_i
        v_ego_estimates = -velocities

        # Outlier rejection when 3+ objects: reject objects whose estimate
        # deviates >2 m/s from the consensus median (removes fast dynamic cars)
        if USE_OUTLIER_REJECTION and len(v_ego_estimates) >= 3:
            median_ego = np.median(v_ego_estimates, axis=0)
            deviations = np.linalg.norm(v_ego_estimates - median_ego, axis=1)
            inlier_mask = deviations <= 2.0
            if inlier_mask.sum() < 1:
                inlier_mask = np.ones(len(v_ego_estimates), dtype=bool)
        else:
            inlier_mask = np.ones(len(v_ego_estimates), dtype=bool)

        w_in = combined_weights[inlier_mask]
        t_ego_cam = np.average(v_ego_estimates[inlier_mask], axis=0,
                               weights=w_in / (w_in.sum() + 1e-9))

        spd_ego = float(np.linalg.norm(t_ego_cam))
        if spd_ego > 15.0 or spd_ego < 0.001: continue

        N_inliers = int(inlier_mask.sum())

        # Rotate to world frame
        R_wi = quat_to_rot(*vio_q[i])
        t_ego_world = R_wi @ R_IMU_CAM @ t_ego_cam

        # Residual = E3_estimate - VIO (m/s)
        residual = t_ego_world - vio_vel[i]
        if float(np.linalg.norm(residual)) > 3.0: continue

        raw_corr[i] = residual
        corr_cnt[i] = max(N_inliers, 1)

    # 4. EMA smooth
    smoothed = np.zeros((n, 3))
    ema = np.zeros(3)
    for i in range(n):
        if corr_cnt[i] > 0:
            w_count = min(corr_cnt[i] / MIN_OBJ_FULL, 1.0)
            ema = EMA_ALPHA * raw_corr[i] * w_count + (1 - EMA_ALPHA) * ema
        else:
            ema *= (1 - EMA_ALPHA * 0.3)
        smoothed[i] = ema

    # 5. Integrate correction
    cumulative = np.zeros((n, 3))
    offset = np.zeros(3)
    for i in range(1, n):
        dt = float(np.clip(vio_t[i] - vio_t[i-1], 0, 0.5))
        
        total_objects_seen = corr_cnt[i] # Inlier count
        inlier_confidence = min(total_objects_seen / 3.0, 1.0) # 3 objects enough
        
        # Dynamic Trust
        dynamic_trust = CORRECTION_TRUST * inlier_confidence
        
        delta = smoothed[i] * dt * dynamic_trust
        mag = float(np.linalg.norm(delta))
        if mag > MAX_CORRECTION * dt:
            delta = delta * (MAX_CORRECTION * dt / mag)
        offset += delta
        cumulative[i] = offset

    return cumulative

def process_run(folder_path, run_id):
    vp  = os.path.join(folder_path, f"vio_path_run_{run_id}.csv")
    op  = os.path.join(folder_path, f"dynamic_objects_run_{run_id}.csv")
    out = os.path.join(folder_path, f"dov_path_run_{run_id}.csv")
    if not os.path.exists(vp): return False

    vio_df = pd.read_csv(vp).dropna().sort_values('timestamp').reset_index(drop=True)
    if len(vio_df) < 10: return False

    obj_df = pd.DataFrame()
    if os.path.exists(op):
        obj_df = pd.read_csv(op).dropna().sort_values('timestamp').reset_index(drop=True)

    n_obj = obj_df['obj_id'].nunique() if not obj_df.empty else 0
    print(f"    Run {run_id}: {len(vio_df)} poses, {len(obj_df)} obj obs, {n_obj} objects")

    if obj_df.empty:
        vio_df[['timestamp','x','y','z']].to_csv(out, index=False)
        return True

    corr = compute_dov_correction(vio_df, obj_df)
    n    = len(vio_df)
    mag  = np.linalg.norm(corr, axis=1)
    act  = int(np.sum(mag > 1e-4))
    
    print(f"    -> Active frames: {act}/{n} ({100*act/n:.0f}%)  "
          f"correction mean={mag.mean():.4f}m  "
          f"max={mag.max():.4f}m  final={mag[-1]:.4f}m")

    dov_xyz = vio_df[['x','y','z']].values + corr
    pd.DataFrame({
        'timestamp': vio_df['timestamp'],
        'x': dov_xyz[:, 0], 'y': dov_xyz[:, 1], 'z': dov_xyz[:, 2]
    }).to_csv(out, index=False)
    
    print(f"    -> Saved {out}")
    return True

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--folder", type=str, default=None)
    a = p.parse_args()
    target = [a.folder] if a.folder else FOLDERS

    print("=" * 60)
    print("DOV-SLAM  —  World-Frame EKF + RANSAC Ego-Motion Residual Corrector")
    print("=" * 60)

    total, ok = 0, 0
    for folder in target:
        fp = os.path.join(BASE_DIR, folder)
        if not os.path.exists(fp): continue
        print(f"\n{'─'*50}\nFolder: {folder}\n{'─'*50}")
        files = glob.glob(os.path.join(fp, "vio_path_run_*.csv"))
        rids  = sorted([int(f.split('_')[-1].split('.')[0]) for f in files])
        for rid in rids:
            total += 1
            if process_run(fp, rid): ok += 1

    print(f"\n{'='*60}\nDone. {ok}/{total} runs processed.\n{'='*60}")