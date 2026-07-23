import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SemanticMasker(Node):
    def __init__(self):
        super().__init__('semantic_masker')
        
        self.bridge = CvBridge()
        
        # All vehicle colors (dynamic + static), all VIODE environments — BGR for OpenCV
        # Dynamic: ids 241-251  |  Static: ids 254-255
        self.dynamic_colors = [
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

        # --- SUBSCRIBERS ---
        # We need the Image to sync timestamps, and Segmentation to create the mask
        self.sub_img0 = self.create_subscription(Image, '/cam0/image_raw', self.img0_cb, 10)
        self.sub_seg0 = self.create_subscription(Image, '/cam0/segmentation', self.seg0_cb, 10)
        
        self.sub_img1 = self.create_subscription(Image, '/cam1/image_raw', self.img1_cb, 10)
        self.sub_seg1 = self.create_subscription(Image, '/cam1/segmentation', self.seg1_cb, 10)

        # --- PUBLISHERS ---
        # Publishes BINARY MASK (255=Ignore, 0=Track)
        self.pub_masked0 = self.create_publisher(Image, '/cam0/masked', 10)
        self.pub_masked1 = self.create_publisher(Image, '/cam1/masked', 10)
        
        self.current_seg0 = None
        self.current_seg1 = None

        self.force_empty        = self.declare_parameter('force_empty',        False).get_parameter_value().bool_value
        self.dilation_kernel    = self.declare_parameter('dilation_kernel',    13   ).get_parameter_value().integer_value
        self.max_mask_fraction  = self.declare_parameter('max_mask_fraction',  0.80 ).get_parameter_value().double_value
        self.get_logger().info(f"Semantic Masker Started. force_empty={self.force_empty} "
                               f"dilation={self.dilation_kernel} max_mask_frac={self.max_mask_fraction}")

        # Startup delay: publish raw frames for the first N seconds
        # so OpenVINS can initialize on clean, unmasked features
        self.startup_delay_sec = 3.0
        self.node_start_time = None   # set on first image received

    def seg0_cb(self, msg):
        try: self.current_seg0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: pass

    def seg1_cb(self, msg):
        try: self.current_seg1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: pass

    def img0_cb(self, msg):
        if self.current_seg0 is not None:
            self.process_and_publish(msg, self.current_seg0, self.pub_masked0)

    def img1_cb(self, msg):
        if self.current_seg1 is not None:
            self.process_and_publish(msg, self.current_seg1, self.pub_masked1)

    def process_and_publish(self, img_msg, seg_img, publisher):
        try:
             # ── Startup delay ──────────────────────────────────────────
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.node_start_time is None:
                self.node_start_time = now
            elapsed = now - self.node_start_time

            if elapsed < self.startup_delay_sec:
                # Publish all-zeros mask (= track everything) during startup
                h, w = img_msg.height, img_msg.width
                empty_mask = np.zeros((h, w), dtype=np.uint8)
                out_msg = self.bridge.cv2_to_imgmsg(empty_mask, encoding='mono8')
                out_msg.header = img_msg.header
                publisher.publish(out_msg)
                return
            # ────────────────────────────────────────────────────────────

            # 1. Resize Segmentation (Safety check)
            # We use the header from img_msg for width/height reference
            if seg_img.shape[:2] != (img_msg.height, img_msg.width):
                seg_img = cv2.resize(seg_img, (img_msg.width, img_msg.height), interpolation=cv2.INTER_NEAREST)

            # 2. Create Binary Mask (0 = Static/Track, 255 = Dynamic/Ignore)
            binary_mask = np.zeros(seg_img.shape[:2], dtype=np.uint8)

            if self.force_empty:
                out_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
                out_msg.header = img_msg.header
                publisher.publish(out_msg)
                return

            # 3. Identify Dynamic Objects
            for color in self.dynamic_colors:
                lower = np.array(color, dtype=np.uint8)
                # Create mask for this specific car color
                color_mask = cv2.inRange(seg_img, lower, lower)
                # Add to main mask (Logical OR)
                binary_mask = cv2.bitwise_or(binary_mask, color_mask)

            # 4. Dilate Mask (Safety Margin)
            # Expand the "Ignore" region by ~5-7 pixels to cover edges
            k = self.dilation_kernel
            kernel = np.ones((k, k), np.uint8)
            binary_mask = cv2.dilate(binary_mask, kernel, iterations=1)

            # 4b. Coverage guard — skip mask if too much of the frame is masked
            if np.count_nonzero(binary_mask) / binary_mask.size > self.max_mask_fraction:
                binary_mask = np.zeros_like(binary_mask)

            # 5. Publish Mask
            out_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
            
            # CRITICAL: Copy timestamp from the CAMERA image so OpenVINS can sync them
            out_msg.header = img_msg.header 
            
            publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing mask: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMasker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()