import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import math
import yaml
import pandas as pd
import os
import sys
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Header

class FastHybridSpeedEstimator(Node):
    def __init__(self, calib_file_path='', debug=True, record_csv=True, run_id="0"):
        super().__init__('fast_hybrid_speed_estimator')
        self.bridge = CvBridge()
        self.debug = debug

        # --- CSV recording ---
        self.record_csv = record_csv
        self.run_id = run_id
        self.csv_data = []  # [timestamp, obj_id, x_cam, y_cam, z_cam]

        # Configurable paths — override with --ros-args -p calib_file:=<path>
        # or -p output_dir:=<path>.  Defaults preserve backward compatibility
        # with existing run scripts that do not pass these parameters.
        calib_file = self.declare_parameter('calib_file', calib_file_path).get_parameter_value().string_value
        self.output_dir = self.declare_parameter('output_dir', '').get_parameter_value().string_value

        self.load_kalibr_params(calib_file)

        self.tracked_colors = [
            (170, 237, 115),   # id=241  city only dynamic
            (239, 169, 255),   # id=242  city only dynamic
            (227, 202,  99),   # id=243  city only dynamic
            (160, 239, 236),   # id=244  city only dynamic
            (243, 234, 143),   # id=245  city only dynamic
            (155, 221, 166),   # id=246  all envs dynamic
            (245, 248, 154),   # id=247  all envs dynamic
            (188, 210, 253),   # id=248  all envs dynamic
            (251,  59, 226),   # id=249  all envs dynamic
            (207,  91, 108),   # id=250  all envs dynamic
            (231, 196, 243),   # id=251  all envs dynamic
            # (181, 231, 209),   # id=254  parking_lot + city_day static
            # (232, 119, 114),   # id=255  all envs static
        ]

        # --- TUNING PARAMETERS (injectable via --ros-args -p name:=value) ---
        self.min_features    = self.declare_parameter('min_features',    12    ).get_parameter_value().integer_value
        self.min_disparity   = self.declare_parameter('min_disparity',   1.2  ).get_parameter_value().double_value
        self.min_mask_area   = self.declare_parameter('min_mask_area',   1000 ).get_parameter_value().integer_value
        orb_nfeatures        = self.declare_parameter('orb_nfeatures',   100  ).get_parameter_value().integer_value
        self.mask_source     = self.declare_parameter('mask_source',     'gt' ).get_parameter_value().string_value

        self.feature_quality = 0.2
        self.ema_alpha = 0.15
        self.time_window_sec = 0.4

        self.smoothed_positions = {}   # color -> smoothed P1_cam (numpy 3D)
        self.pos_ema_alpha = 0.3       # how fast position estimate updates

        self.prev_data = None
        self.object_history = {}
        self.smoothed_speeds = {}
        self.tracking_modes = {}
        self.latest_ego_pose = None

        self.orb = cv2.ORB_create(nfeatures=orb_nfeatures, scaleFactor=1.2, nlevels=4)
        self.get_logger().info(f"Hybrid Estimator params: min_features={self.min_features} "
                               f"min_disparity={self.min_disparity} orb_nfeatures={orb_nfeatures} "
                               f"min_mask_area={self.min_mask_area}")
        self.bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/ov_msckf/poseimu', self.pose_cb, 10)
        if self.debug: self.pub_debug = self.create_publisher(Image, '/speed_estimator/debug_image', 10)
        self.pub_obj_vel = self.create_publisher(TwistWithCovarianceStamped, '/dov/object_velocities', 10)

        self.sub_img0 = message_filters.Subscriber(self, Image, '/cam0/image_raw')
        self.sub_img1 = message_filters.Subscriber(self, Image, '/cam1/image_raw')

        if self.mask_source == 'yolo':
            # Use all-detections masks from yolo_masker (includes parked vehicles as DOV anchors)
            self.sub_seg0 = message_filters.Subscriber(self, Image, '/cam0/objects')
            self.sub_seg1 = message_filters.Subscriber(self, Image, '/cam1/objects')
        else:
            # Use VIODE ground-truth segmentation color IDs
            self.sub_seg0 = message_filters.Subscriber(self, Image, '/cam0/segmentation')
            self.sub_seg1 = message_filters.Subscriber(self, Image, '/cam1/segmentation')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_img0, self.sub_seg0, self.sub_img1, self.sub_seg1], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info(
            f"Fast Time-Windowed Hybrid Estimator Started. "
            f"mask_source={self.mask_source} CSV Recording: {self.record_csv}"
        )

    def load_kalibr_params(self, filepath):
        try:
            with open(filepath, 'r') as file:
                clean_lines = [line for line in file.readlines() if not line.strip().startswith('%YAML')]
                calib = yaml.safe_load(''.join(clean_lines))
                
            self.fx, self.fy, self.cx, self.cy = calib['cam0']['intrinsics']
            self.baseline = abs(calib['cam1']['T_imu_cam'][1][3] - calib['cam0']['T_imu_cam'][1][3])
            self.T_imu_cam0 = np.array(calib['cam0']['T_imu_cam'])
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise e

    def pose_cb(self, msg):
        self.latest_ego_pose = msg.pose.pose

    def quat_to_rot_matrix(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
            [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])

    def transform_to_global(self, P_cam, pose_msg):
        if pose_msg is None: return P_cam
        P_cam_homo = np.array([P_cam[0], P_cam[1], P_cam[2], 1.0])
        P_imu = np.dot(self.T_imu_cam0, P_cam_homo)
        
        R = self.quat_to_rot_matrix(pose_msg.orientation)
        T = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        return np.dot(R, P_imu[0:3]) + T

    def _mask_centroid_binary(self, mask):
        """Centroid of a binary mask (uint8, 255=object). Returns (cx, cy) or None."""
        M = cv2.moments(mask)
        if M["m00"] > 100:
            return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return None

    def get_mask_centroid(self, seg_img, color):
        mask = cv2.inRange(seg_img, np.array(color), np.array(color))
        M = cv2.moments(mask)
        if M["m00"] > 100:
            return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return None

    def calc_3d_point(self, u, v, disparity):
        Z = (self.fx * self.baseline) / disparity
        X = ((u - self.cx) * Z) / self.fx
        Y = ((v - self.cy) * Z) / self.fy
        return np.array([X, Y, Z])

    def _get_object_masks(self, seg0, seg1):
        """
        Returns list of (obj_id, mask_left, mask_right) for each detected object.

        GT mode  : iterates VIODE color IDs — one color = one object.
        YOLO mode: decodes per-object label map from yolo_masker (/cam0/objects).
                   Pixel value N = detection index N (1-indexed; 0 = background).
                   Each unique non-zero label is one segmented object.
                   Right-camera mask is the full non-zero region of cam1 label map
                   (epipolar constraint in the stereo matcher handles cross-matching).
        """
        if self.mask_source != 'yolo':
            return [
                (color,
                 cv2.inRange(seg0, np.array(color), np.array(color)),
                 cv2.inRange(seg1, np.array(color), np.array(color)))
                for color in self.tracked_colors
            ]

        # seg0/seg1 are mono8 label maps from yolo_masker /cam0/objects
        label0 = seg0 if seg0.ndim == 2 else cv2.cvtColor(seg0, cv2.COLOR_BGR2GRAY)
        label1 = seg1 if seg1.ndim == 2 else cv2.cvtColor(seg1, cv2.COLOR_BGR2GRAY)

        # Right-cam search region: all non-background pixels, slightly dilated
        mask_r_full = (label1 > 0).astype(np.uint8) * 255
        mask_r_full = cv2.dilate(mask_r_full, np.ones((5, 5), np.uint8), iterations=1)

        results = []
        for label_val in np.unique(label0):
            if label_val == 0:
                continue  # background
            mask_l = (label0 == label_val).astype(np.uint8) * 255
            if cv2.countNonZero(mask_l) < self.min_mask_area:
                continue
            # Centroid rounded to 30 px grid → stable ID across frames
            M = cv2.moments(mask_l)
            if M['m00'] == 0:
                continue
            cx = int(round((M['m10'] / M['m00']) / 30.0)) * 30
            cy = int(round((M['m01'] / M['m00']) / 30.0)) * 30
            obj_id = (cx, cy)
            results.append((obj_id, mask_l, mask_r_full))
        return results

    def sync_callback(self, img0_msg, seg0_msg, img1_msg, seg1_msg):
        try:
            img0_bgr = self.bridge.imgmsg_to_cv2(img0_msg, desired_encoding='bgr8')
            img0_gray = cv2.cvtColor(img0_bgr, cv2.COLOR_BGR2GRAY)
            img1_gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(img1_msg, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)

            if self.mask_source == 'yolo':
                seg0_bgr = self.bridge.imgmsg_to_cv2(seg0_msg, desired_encoding='mono8')
                seg1_bgr = self.bridge.imgmsg_to_cv2(seg1_msg, desired_encoding='mono8')
            else:
                seg0_bgr = self.bridge.imgmsg_to_cv2(seg0_msg, desired_encoding='bgr8')
                seg1_bgr = self.bridge.imgmsg_to_cv2(seg1_msg, desired_encoding='bgr8')
            
            curr_time = img0_msg.header.stamp.sec + (img0_msg.header.stamp.nanosec * 1e-9)
            curr_pose = self.latest_ego_pose

            if self.prev_data is None:
                self.prev_data = (img0_gray, img1_gray, seg0_bgr, seg1_bgr, curr_time, curr_pose)
                return

            prev_img0, prev_img1, prev_seg0, prev_seg1, prev_time, prev_pose = self.prev_data
            dt_frame = curr_time - prev_time

            speeds_to_draw = {}
            seen_colors = []
            features_to_draw = []
            centroids_to_draw = []

            if dt_frame > 0:
                for obj_id, mask_curr, mask_curr_r in self._get_object_masks(seg0_bgr, seg1_bgr):

                    if cv2.countNonZero(mask_curr) < self.min_mask_area: continue

                    tracking_mode = None
                    P1_global_current = None
                    P1_cam_current = None

                    # ==========================================
                    # 1. ORB STEREO FEATURE TRACKING
                    # ==========================================

                    kp1, des1 = self.orb.detectAndCompute(img0_gray, mask=mask_curr)

                    if des1 is not None and len(kp1) >= self.min_features:
                        if self.mask_source != 'yolo':
                            # GT mode: per-object right mask (same color in right cam)
                            kernel = np.ones((5, 5), np.uint8)
                            mask_curr_r = cv2.dilate(mask_curr_r, kernel, iterations=1)

                        kp1_r, des1_r = self.orb.detectAndCompute(img1_gray, mask=mask_curr_r)
                        stereo_matches = []
                        if des1_r is not None and len(des1_r) >= 2:
                            matches_lr = self.bf.match(des1, des1_r)
                            for m in matches_lr:
                                pt_l = kp1[m.queryIdx].pt
                                pt_r = kp1_r[m.trainIdx].pt
                                # Epipolar constraint: same row ±2px
                                if abs(pt_l[1] - pt_r[1]) < 2.0:
                                    disp = pt_l[0] - pt_r[0]
                                    if self.min_disparity <= disp <= 60.0:  # ~0.3m minimum depth
                                        stereo_matches.append((pt_l, disp))

                        if len(stereo_matches) >= self.min_features // 2:   
                              cam_pts = [self.calc_3d_point(pt[0], pt[1], d)    
                                      for (pt, d) in stereo_matches]            
                              depths = [p[2] for p in cam_pts]                  
                              if np.std(depths) <= 2.0:                         
                                  global_pts = [self.transform_to_global(p, curr_pose) for p in cam_pts]                                                  
                                  P1_cam_current    = np.median(cam_pts, axis=0)                                                                       
                                  P1_global_current = np.median(global_pts, axis=0)                                                                       
                                  tracking_mode = "ORB"
                                  features_to_draw.extend(
                                      [(int(pt[0]), int(pt[1])) for (pt, d) in stereo_matches[:20]])   

                    # ==========================================
                    # 2. FALLBACK TO CENTROID TRACKING
                    # ==========================================
                    if tracking_mode is None:
                        if self.mask_source == 'yolo':
                            c1_L = self._mask_centroid_binary(mask_curr)
                            c1_R = self._mask_centroid_binary(mask_curr_r)
                        else:
                            c1_L = self.get_mask_centroid(seg0_bgr, obj_id)
                            c1_R = self.get_mask_centroid(seg1_bgr, obj_id)

                        if c1_L and c1_R:
                            disp1 = c1_L[0] - c1_R[0]
                            if abs(c1_L[1] - c1_R[1]) < 2.0 and disp1 >= self.min_disparity:
                                P1_cam_current = self.calc_3d_point(c1_L[0], c1_L[1], disp1)
                                P1_global_current = self.transform_to_global(P1_cam_current, curr_pose)
                                tracking_mode = "C"
                                centroids_to_draw.append(c1_L)

                    # ==========================================
                    # 3. Record position to CSV
                    # ==========================================
                    if self.record_csv and P1_cam_current is not None:
                        obj_id_str = f"{obj_id[0]}_{obj_id[1]}_{obj_id[2]}" if isinstance(obj_id, tuple) and len(obj_id) == 3 else f"{obj_id[0]}_{obj_id[1]}"
                        self.csv_data.append([curr_time, obj_id_str, P1_cam_current[0], P1_cam_current[1], P1_cam_current[2]])

                    # ==========================================
                    # 4. Time-windowed speed calculation (for debug visualization)
                    # ==========================================
                    if P1_global_current is not None:
                        seen_colors.append(obj_id)
                        self.tracking_modes[obj_id] = tracking_mode

                        if obj_id not in self.object_history:
                            self.object_history[obj_id] = []
                        self.object_history[obj_id].append((P1_global_current, curr_time))

                        while len(self.object_history[obj_id]) > 1 and (curr_time - self.object_history[obj_id][0][1]) > self.time_window_sec:
                            self.object_history[obj_id].pop(0)

                        history = self.object_history[obj_id]

                        if len(history) >= 2 and (curr_time - history[0][1]) > 0.15:
                            dt_window = curr_time - history[0][1]

                            delta_pos_world = P1_global_current - history[0][0]
                            v_world = delta_pos_world / dt_window

                            if curr_pose is not None:
                                R_wc = self.quat_to_rot_matrix(curr_pose.orientation)
                                T_cam_imu = self.T_imu_cam0[:3, :3].T
                                R_wc_cam = R_wc @ np.linalg.inv(T_cam_imu)
                                v_cam = R_wc_cam.T @ v_world
                            else:
                                v_cam = np.zeros(3)

                            if len(history) >= 4:
                                positions = np.array([h[0] for h in history])
                                pos_var = np.var(positions, axis=0)
                                vel_var = np.clip(pos_var / (dt_window ** 2), 0.01, 4.0)
                            else:
                                vel_var = np.array([1.0, 1.0, 1.0])

                            twist_msg = TwistWithCovarianceStamped()
                            twist_msg.header.stamp = img0_msg.header.stamp
                            twist_msg.header.frame_id = f"obj_{obj_id[0]}_{obj_id[1]}"
                            twist_msg.twist.twist.linear.x = float(v_cam[0])
                            twist_msg.twist.twist.linear.y = float(v_cam[1])
                            twist_msg.twist.twist.linear.z = float(v_cam[2])
                            cov = [0.0] * 36
                            cov[0]  = float(vel_var[0])
                            cov[7]  = float(vel_var[1])
                            cov[14] = float(vel_var[2])
                            twist_msg.twist.covariance = cov
                            self.pub_obj_vel.publish(twist_msg)

                        if len(history) >= 2 and (curr_time - history[0][1]) > 0.15:
                            dt_window = curr_time - history[0][1]
                            dist_window = np.linalg.norm(P1_global_current - history[0][0])
                            raw_speed_kmh = (dist_window / dt_window) * 3.6

                            if obj_id in self.smoothed_speeds:
                                self.smoothed_speeds[obj_id] = (self.ema_alpha * raw_speed_kmh) + ((1.0 - self.ema_alpha) * self.smoothed_speeds[obj_id])
                            else:
                                self.smoothed_speeds[obj_id] = raw_speed_kmh

                            speeds_to_draw[obj_id] = self.smoothed_speeds[obj_id]
                        elif obj_id in self.smoothed_speeds:
                            speeds_to_draw[obj_id] = self.smoothed_speeds[obj_id]

            for oid in list(self.smoothed_speeds.keys()):
                if oid not in seen_colors:
                    del self.smoothed_speeds[oid]
                    del self.tracking_modes[oid]
                    self.smoothed_positions.pop(str(oid), None)
                    if oid in self.object_history:
                        del self.object_history[oid]

            self.prev_data = (img0_gray, img1_gray, seg0_bgr, seg1_bgr, curr_time, curr_pose)

            # --- DEBUG VISUALIZATION ---
            if self.debug:
                if self.mask_source == 'yolo':
                    seg_vis = cv2.cvtColor(seg0_bgr, cv2.COLOR_GRAY2BGR) if seg0_bgr.ndim == 2 else seg0_bgr
                else:
                    seg_vis = seg0_bgr
                overlay = cv2.addWeighted(img0_bgr, 0.5, seg_vis, 0.5, 0)
                for pt in features_to_draw: cv2.circle(overlay, pt, 3, (0, 255, 255), -1)
                for pt in centroids_to_draw: cv2.circle(overlay, pt, 6, (0, 0, 255), -1)

                for oid, speed in speeds_to_draw.items():
                    mode = self.tracking_modes.get(oid, "?")
                    draw_pt = (oid[0], oid[1]) if not isinstance(oid, tuple) or len(oid) == 2 else None
                    if draw_pt:
                        cv2.putText(overlay, f"{speed:.1f} km/h ({mode})", (draw_pt[0], draw_pt[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                debug_msg.header = img0_msg.header
                self.pub_debug.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def save_csv_data(self):
        if not self.record_csv or not self.csv_data:
            return

        save_dir = self.output_dir if self.output_dir else os.path.expanduser('~/ov_results')
        os.makedirs(save_dir, exist_ok=True)
        filename = os.path.join(save_dir, f"dynamic_objects_run_{self.run_id}.csv")

        df = pd.DataFrame(self.csv_data, columns=['timestamp', 'obj_id', 'x_cam', 'y_cam', 'z_cam'])
        df.to_csv(filename, index=False)
        print(f"[hybrid_speed_estimator] Saved dynamic object positions: {filename} ({len(self.csv_data)} rows)")

def main(args=None):
    rclpy.init(args=args)

    # run_id from positional argv for bash script compatibility
    run_id = sys.argv[1] if len(sys.argv) > 1 else "0"

    # calib_file and output_dir are set via --ros-args -p calib_file:=<path>
    # (passed by the run scripts or launch files; no default here).
    node = FastHybridSpeedEstimator(calib_file_path='', debug=True, record_csv=True, run_id=run_id)
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save_csv_data()
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except:
            pass

if __name__ == '__main__':
    main()