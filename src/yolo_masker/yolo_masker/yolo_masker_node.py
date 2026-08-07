"""
YOLOv11-based dynamic object masker with ego-motion-compensated optical flow.

Static vs Dynamic discrimination (the core fix):
  For each YOLO detection, pixel-level optical flow is compared against background
  ego-motion (estimated from non-car feature points). A vehicle whose flow matches
  the background is STATIC in world frame — it is left unmasked for VIO features
  but still included in the DOV label map. A vehicle whose flow deviates from the
  background is DYNAMIC — it is masked for VIO.

  This mirrors exactly what the GT semantic_masker does (masks dynamic IDs 241-251,
  leaves static parked cars 254-255 unmasked) but with no dataset-specific colours,
  making it suitable for a real autonomous shuttle.

Output topics
─────────────
/cam0/masked   (mono8)  VIO mask: 255=ignore, 0=track.
                         Only DYNAMIC detections (flow residual > threshold).
                         Empty (all-zeros) when coverage > max_mask_fraction.

/cam0/objects  (mono8)  Per-object label map for hybrid_speed_estimator (DOV).
                         ALL detections regardless of static/dynamic classification.
                         Pixel value = detection index + 1  (0 = background).
                         No coverage guard.

Parameters
──────────
model_path              path to yolo11n-seg.pt (absolute)
classes_to_mask         COCO class IDs to consider  [0,1,2,3,5,6,7]
confidence_threshold    YOLO confidence cutoff       0.25
dilation_kernel         mask dilation size (px)      13
max_mask_fraction       VIO coverage guard           0.80
force_empty             publish all-zeros (unmasked ablation)  False
use_flow_classifier     enable static/dynamic discrimination   True
flow_dynamic_threshold  residual flow (px) to call dynamic     2.0
flow_min_features       min KLT features per object to decide  5

Threading model
───────────────
YOLO inference (~100ms per stereo pair) runs in a daemon background thread so
the ROS2 executor is never blocked.  _stereo_cb publishes the most-recent cached
masks immediately (<1ms) and signals the background thread with the latest frame
pair.  If YOLO is still processing when the next pair arrives, the old queued
pair is replaced — YOLO always processes the newest available frame.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


# COCO class IDs  0=person 1=bicycle 2=car 3=motorcycle 5=bus 6=train 7=truck
_DEFAULT_CLASSES = [0, 1, 2, 3, 5, 6, 7]

_LK_PARAMS = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)


class _FlowClassifier:
    """
    Per-camera state for ego-motion-compensated optical flow classification.

    Keeps the previous grayscale frame and computes:
      1. Background ego-motion: median flow of feature points NOT on any detection.
      2. Per-object flow: mean flow of feature points inside each segmentation mask.
      3. Classification: |obj_flow - bg_flow| > threshold → dynamic.

    Falls back to True (dynamic) when:
      - First frame (no previous frame yet).
      - Fewer than flow_min_features tracked inside the object mask.
      - Background features too sparse to estimate ego-motion reliably.
    """

    def __init__(self):
        self.prev_gray: np.ndarray | None = None

    def prime(self, gray: np.ndarray) -> None:
        """Update previous frame without running classification (use during startup)."""
        self.prev_gray = gray.copy()

    def classify(
        self,
        gray_cur: np.ndarray,
        binary_masks: list[np.ndarray],
        threshold: float,
        min_features: int,
    ) -> list[bool]:
        """
        Returns a list[bool] aligned with binary_masks.
        True = dynamic (mask for VIO), False = static (preserve for VIO features).
        Updates prev_gray.
        """
        n = len(binary_masks)

        if self.prev_gray is None or n == 0:
            self.prev_gray = gray_cur.copy()
            return [True] * n

        # ── Background mask: everything outside ALL detections (dilated margin) ──
        combined = np.zeros_like(binary_masks[0])
        for m in binary_masks:
            combined = cv2.bitwise_or(combined, m)
        bg_mask = (cv2.dilate(combined, np.ones((25, 25), np.uint8)) == 0).astype(np.uint8) * 255

        # ── Background ego-motion (median of tracked feature flows) ────────────
        bg_pts = cv2.goodFeaturesToTrack(
            self.prev_gray, maxCorners=150, qualityLevel=0.01,
            minDistance=8, mask=bg_mask,
        )
        bg_flow = np.zeros(2, dtype=np.float32)
        bg_reliable = False
        if bg_pts is not None and len(bg_pts) >= 5:
            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray_cur, bg_pts, None, **_LK_PARAMS)
            good = st.ravel() == 1
            if good.sum() >= 3:
                delta = (nxt[good] - bg_pts[good]).reshape(-1, 2)
                bg_flow = np.median(delta, axis=0)
                bg_reliable = True

        # ── Per-object classification ─────────────────────────────────────────
        results: list[bool] = []
        for mask in binary_masks:
            if not bg_reliable:
                results.append(True)
                continue

            obj_pts = cv2.goodFeaturesToTrack(
                self.prev_gray, maxCorners=30, qualityLevel=0.01,
                minDistance=5, mask=mask,
            )
            if obj_pts is None or len(obj_pts) < min_features:
                results.append(True)   # conservative: too few texture → mask it
                continue

            nxt, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray_cur, obj_pts, None, **_LK_PARAMS)
            good = st.ravel() == 1
            if good.sum() < min_features:
                results.append(True)
                continue

            delta = (nxt[good] - obj_pts[good]).reshape(-1, 2)
            obj_flow = delta.mean(axis=0)
            residual = float(np.linalg.norm(obj_flow - bg_flow))
            results.append(residual > threshold)

        self.prev_gray = gray_cur.copy()
        return results


class YoloMasker(Node):
    def __init__(self):
        super().__init__('yolo_masker')
        self.bridge = CvBridge()

        self.model_path             = self.declare_parameter('model_path',             '').get_parameter_value().string_value
        self.classes_to_mask        = list(self.declare_parameter('classes_to_mask',   _DEFAULT_CLASSES).get_parameter_value().integer_array_value)
        self.confidence_threshold   = self.declare_parameter('confidence_threshold',   0.25 ).get_parameter_value().double_value
        self.dilation_kernel        = self.declare_parameter('dilation_kernel',        13   ).get_parameter_value().integer_value
        self.max_mask_fraction      = self.declare_parameter('max_mask_fraction',      0.80 ).get_parameter_value().double_value
        self.force_empty            = self.declare_parameter('force_empty',            False).get_parameter_value().bool_value
        self.use_flow_classifier    = self.declare_parameter('use_flow_classifier',    True ).get_parameter_value().bool_value
        self.flow_dynamic_threshold = self.declare_parameter('flow_dynamic_threshold', 2.0  ).get_parameter_value().double_value
        self.flow_min_features      = self.declare_parameter('flow_min_features',      5    ).get_parameter_value().integer_value
        self.use_clahe              = self.declare_parameter('use_clahe',              True ).get_parameter_value().bool_value
        self.clahe_clip_limit       = self.declare_parameter('clahe_clip_limit',       2.0  ).get_parameter_value().double_value
        self.device                 = self.declare_parameter('device',                 'cuda').get_parameter_value().string_value

        # CLAHE applied to BGR before YOLO inference — same enhancement OpenVINS
        # uses internally (histogram_method: CLAHE). Improves night-time detection
        # where YOLO is otherwise blind, preventing corrupt features from leaking to VIO.
        self._clahe = cv2.createCLAHE(clipLimit=self.clahe_clip_limit, tileGridSize=(8, 8))

        self._startup_delay_sec = 3.0
        self._node_start_time   = None

        self._flow_clf0 = _FlowClassifier()
        self._flow_clf1 = _FlowClassifier()

        # ── Async YOLO state ──────────────────────────────────────────────────
        # _cache0/1: most recent (vio_mask, label_map) produced by YOLO thread.
        # None until the first YOLO result is ready; _stereo_cb falls back to
        # all-zeros while the cache is empty.
        self._cache_lock = threading.Lock()
        self._cache0: tuple | None = None   # (vio_mask np.ndarray, label_map np.ndarray)
        self._cache1: tuple | None = None

        # _latest_pair: most recent stereo pair waiting for YOLO processing.
        # Replaced (not queued) when a new pair arrives — YOLO always sees the
        # freshest frame, never builds a backlog.
        self._latest_lock = threading.Lock()
        self._latest_pair: tuple | None = None   # (msg0, msg1) or None
        self._new_pair_event = threading.Event()

        # Latched publisher so bag-play scripts can wait for the ready signal
        _latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._ready_pub = self.create_publisher(Bool, '/yolo_masker/ready', _latched_qos)

        self._model = None
        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error("ultralytics not installed — run: pip install ultralytics")
        elif not self.model_path:
            self.get_logger().error("model_path parameter is empty — set it with: --ros-args -p model_path:=<path/to/yolo.pt>")
        else:
            self.get_logger().info(f"Loading YOLO segmentation model: {self.model_path} (device={self.device})")
            self._model = YOLO(self.model_path)
            self._model.to(self.device)
            # Warm up to eliminate first-frame latency spike
            _dummy = np.zeros((480, 752, 3), dtype=np.uint8)
            self._model(_dummy, classes=self.classes_to_mask, conf=self.confidence_threshold, verbose=False)
            self.get_logger().info(f"YOLO model loaded and warmed up on {self.device}.")

        # Start background YOLO thread before subscribing so it's ready immediately
        self._yolo_thread = threading.Thread(target=self._yolo_loop, daemon=True)
        self._yolo_thread.start()

        # Synchronize cam0+cam1 so masks are always published as a matched pair.
        _sub0 = message_filters.Subscriber(self, Image, '/cam0/image_raw')
        _sub1 = message_filters.Subscriber(self, Image, '/cam1/image_raw')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [_sub0, _sub1], queue_size=2, slop=0.05,
        )
        self._sync.registerCallback(self._stereo_cb)

        self._pub0         = self.create_publisher(Image, '/cam0/masked',  10)
        self._pub1         = self.create_publisher(Image, '/cam1/masked',  10)
        self._pub0_objects = self.create_publisher(Image, '/cam0/objects', 10)
        self._pub1_objects = self.create_publisher(Image, '/cam1/objects', 10)

        # Signal readiness so bag-play scripts know the GPU is warm
        ready_msg = Bool()
        ready_msg.data = True
        self._ready_pub.publish(ready_msg)

        self.get_logger().info(
            f"YoloMasker ready — model={self.model_path} force_empty={self.force_empty} "
            f"conf={self.confidence_threshold} dilation={self.dilation_kernel} "
            f"max_mask={self.max_mask_fraction:.0%} "
            f"clahe={'ON clip=' + str(self.clahe_clip_limit) if self.use_clahe else 'OFF'} "
            f"flow={'ON' if self.use_flow_classifier else 'OFF'} "
            f"flow_thresh={self.flow_dynamic_threshold}px"
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _in_startup(self) -> bool:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._node_start_time is None:
            self._node_start_time = now
        return (now - self._node_start_time) < self._startup_delay_sec

    def _decode(self, msg: Image) -> np.ndarray | None:
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge decode error: {e}")
            return None

    def _pub_pair(self, vio_mask, label_map, header, vio_pub, obj_pub):
        vio_out = self.bridge.cv2_to_imgmsg(vio_mask, encoding='mono8')
        vio_out.header = header
        vio_pub.publish(vio_out)
        obj_out = self.bridge.cv2_to_imgmsg(label_map, encoding='mono8')
        obj_out.header = header
        obj_pub.publish(obj_out)

    # ── ROS callback (executor thread — must not block) ───────────────────────

    def _stereo_cb(self, msg0: Image, msg1: Image):
        """
        Publishes cached YOLO masks immediately with the current frame timestamp.
        Falls back to all-zeros while the cache is empty (startup / force_empty).
        Signals the background YOLO thread with the newest stereo pair.
        """
        h, w = msg0.height, msg0.width
        empty = np.zeros((h, w), dtype=np.uint8)

        # Grab cached results under lock (reference copy — arrays are never mutated)
        with self._cache_lock:
            vio0, lbl0 = self._cache0 if self._cache0 is not None else (empty, empty)
            vio1, lbl1 = self._cache1 if self._cache1 is not None else (empty, empty)

        if self.force_empty:
            # VIO mask is all-zeros (unmasked control condition).
            # Objects label map still carries YOLO detections so the DOV speed
            # estimator can track vehicles in the unmasked runs too.
            self._pub_pair(empty, lbl0, msg0.header, self._pub0, self._pub0_objects)
            self._pub_pair(empty, lbl1, msg1.header, self._pub1, self._pub1_objects)
        else:
            self._pub_pair(vio0, lbl0, msg0.header, self._pub0, self._pub0_objects)
            self._pub_pair(vio1, lbl1, msg1.header, self._pub1, self._pub1_objects)

        # Hand the latest pair to YOLO thread (replace any unprocessed older pair)
        with self._latest_lock:
            self._latest_pair = (msg0, msg1)
        self._new_pair_event.set()

    # ── YOLO background thread ────────────────────────────────────────────────

    def _yolo_loop(self):
        """
        Runs YOLO inference in a background daemon thread.

        Waits for _new_pair_event, grabs the latest pair, processes both cameras,
        and updates the shared cache.  Always processes the newest available frame —
        if a new pair arrives while YOLO is busy, the old queued pair is silently
        replaced, preventing backlog accumulation.
        """
        while rclpy.ok():
            fired = self._new_pair_event.wait(timeout=1.0)
            if not fired:
                continue
            self._new_pair_event.clear()

            with self._latest_lock:
                pair = self._latest_pair
                self._latest_pair = None
            if pair is None:
                continue

            msg0, msg1 = pair
            in_startup = self._in_startup()

            bgr0 = self._decode(msg0)
            bgr1 = self._decode(msg1)

            if in_startup:
                if bgr0 is not None:
                    self._flow_clf0.prime(cv2.cvtColor(bgr0, cv2.COLOR_BGR2GRAY))
                if bgr1 is not None:
                    self._flow_clf1.prime(cv2.cvtColor(bgr1, cv2.COLOR_BGR2GRAY))
            else:
                # Single batched YOLO inference for both cameras, then update cache
                # atomically so _stereo_cb always sees a consistent stereo pair.
                result0, result1 = self._build_masks_stereo(bgr0, bgr1)
                with self._cache_lock:
                    if result0 is not None:
                        self._cache0 = result0
                    if result1 is not None:
                        self._cache1 = result1

    # ── Core YOLO + flow processing (called only from _yolo_loop) ────────────

    def _build_masks_stereo(
        self,
        bgr0: np.ndarray | None,
        bgr1: np.ndarray | None,
    ) -> tuple:
        """
        Run a single batched YOLO inference on both cameras, then apply the
        flow classifier independently per camera.

        Returns (result0, result1) where each result is (vio_mask, label_map)
        or None if the corresponding bgr was None.

        Single batched inference means:
          - One GPU kernel launch instead of two (faster).
          - Both cameras processed at the exact same instant (no gap between
            cam0 and cam1 YOLO results, zero temporal mismatch).
        """
        inputs = []
        valid = []   # which cameras have valid BGR
        for bgr in (bgr0, bgr1):
            if bgr is not None:
                if self.use_clahe:
                    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
                    lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
                    inputs.append(cv2.cvtColor(lab, cv2.COLOR_LAB2BGR))
                else:
                    inputs.append(bgr)
                valid.append(True)
            else:
                valid.append(False)

        if not inputs:
            return None, None

        all_results = self._model(
            inputs,
            classes=self.classes_to_mask,
            conf=self.confidence_threshold,
            verbose=False,
        )

        out = [None, None]
        result_idx = 0
        for cam_idx, (bgr, is_valid) in enumerate(zip((bgr0, bgr1), valid)):
            if not is_valid:
                continue
            flow_clf = self._flow_clf0 if cam_idx == 0 else self._flow_clf1
            out[cam_idx] = self._process_detections(
                bgr, all_results[result_idx], flow_clf
            )
            result_idx += 1

        return out[0], out[1]

    def _process_detections(
        self,
        bgr: np.ndarray,
        yolo_result,
        flow_clf: _FlowClassifier,
    ) -> tuple:
        """
        Convert one camera's YOLO result into (vio_mask, label_map).

        vio_mask  : only DYNAMIC detections, dilated.
                    All-zeros if coverage > max_mask_fraction.
        label_map : ALL detections (static + dynamic), 1-indexed.
                    No coverage guard — DOV sees every object.
        """
        h, w = bgr.shape[:2]
        kernel = np.ones((self.dilation_kernel, self.dilation_kernel), np.uint8)
        vio_mask  = np.zeros((h, w), dtype=np.uint8)
        label_map = np.zeros((h, w), dtype=np.uint8)

        if yolo_result.masks is None or len(yolo_result.masks) == 0:
            flow_clf.prime(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
            return vio_mask, label_map

        seg_np = yolo_result.masks.data.cpu().numpy()   # (N, Hm, Wm) float32
        n_dets = seg_np.shape[0]

        binary_masks = []
        for i in range(n_dets):
            m = cv2.resize(seg_np[i], (w, h), interpolation=cv2.INTER_LINEAR)
            binary_masks.append((m > 0.5).astype(np.uint8))

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.use_flow_classifier:
            is_dynamic = flow_clf.classify(
                gray, binary_masks,
                self.flow_dynamic_threshold,
                self.flow_min_features,
            )
        else:
            flow_clf.prime(gray)
            is_dynamic = [True] * n_dets

        for i, binary in enumerate(binary_masks):
            label_map[binary == 1] = i + 1
            if is_dynamic[i]:
                vio_mask[binary == 1] = 255

        vio_mask = cv2.dilate(vio_mask, kernel, iterations=1)

        if np.count_nonzero(vio_mask) / vio_mask.size > self.max_mask_fraction:
            vio_mask = np.zeros((h, w), dtype=np.uint8)

        return vio_mask, label_map


def main(args=None):
    rclpy.init(args=args)
    node = YoloMasker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
