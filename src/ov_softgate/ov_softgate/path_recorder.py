import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Bool  # Import Bool for the trigger
import pandas as pd
import sys
import os
from nav_msgs.msg import Odometry

class PathRecorder(Node):
    def __init__(self, run_id):
        super().__init__('path_recorder')
        self.run_id = run_id
        self.vio_path_msg = None
        self.gt_path_msg = None
        # Override with: --ros-args -p output_dir:=<absolute_path>
        self.output_dir = self.declare_parameter('output_dir', '').get_parameter_value().string_value

        # Subscribers for Paths
        self.create_subscription(Path, '/ov_msckf/pathimu', self.vio_cb, 10)
        self.create_subscription(Path, '/gt_path', self.gt_cb, 10)

        # NEW: Save Trigger Subscriber
        self.create_subscription(Bool, '/recorder/save', self.save_trigger_cb, 10)

        self.dov_odom_poses = []  # accumulate corrected odom as poses
        self.create_subscription(Odometry, '/dov/corrected_odometry', self.dov_cb, 20)
        
        self.get_logger().info(f"Recorder started for Run #{run_id}...")

    def vio_cb(self, msg):
        self.vio_path_msg = msg

    def gt_cb(self, msg):
        self.gt_path_msg = msg
    
    def dov_cb(self, msg):
        self.dov_odom_poses.append({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
        })

    # NEW: Callback to force save when message received
    def save_trigger_cb(self, msg):
        if msg.data:
            self.save_to_csv()

    def save_to_csv(self):
        self.get_logger().info(f"Saving CSV data (run_id={self.run_id})...")

        save_folder = self.output_dir if self.output_dir else os.path.expanduser('~/ov_results')
        os.makedirs(save_folder, exist_ok=True)

        try:
            # 1. Save VIO Path
            if self.vio_path_msg:
                data = []
                for pose in self.vio_path_msg.poses:
                    data.append({
                        'timestamp': pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9,
                        'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z,
                        'qx': pose.pose.orientation.x, 'qy': pose.pose.orientation.y, 'qz': pose.pose.orientation.z, 'qw': pose.pose.orientation.w
                    })
                filename = os.path.join(save_folder, f"vio_path_run_{self.run_id}.csv")
                pd.DataFrame(data).to_csv(filename, index=False)
                self.get_logger().info(f"Saved {filename}")
            else:
                self.get_logger().warn("No VIO path data received!")

            # 2. Save GT Path
            if self.gt_path_msg:
                data = []
                for pose in self.gt_path_msg.poses:
                    data.append({
                        'timestamp': pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9,
                        'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z,
                        'qx': pose.pose.orientation.x, 'qy': pose.pose.orientation.y, 'qz': pose.pose.orientation.z, 'qw': pose.pose.orientation.w
                    })
                filename = os.path.join(save_folder, f"gt_path_run_{self.run_id}.csv")
                pd.DataFrame(data).to_csv(filename, index=False)
                self.get_logger().info(f"Saved {filename}")
            else:
                self.get_logger().warn("No GT path data received!")

            if self.dov_odom_poses:
                filename = os.path.join(save_folder, f"dov_path_run_{self.run_id}.csv")
                pd.DataFrame(self.dov_odom_poses).to_csv(filename, index=False)
                self.get_logger().info(f"Saved {filename}")
        except Exception as e:
            self.get_logger().error(f"save_to_csv FAILED (run_id={self.run_id!r}): {e}")

def main(args=None):
    # Take the first positional arg before --ros-args as the run_id.
    # Iterating the full sys.argv and keeping the last match breaks when
    # --ros-args -p key:=value is present, because key:=value overwrites
    # the actual run number.
    run_id = "0"
    for arg in sys.argv[1:]:
        if arg == '--ros-args':
            break
        if not arg.startswith('-') and not arg.startswith('/') and not arg.endswith('.py'):
            run_id = arg
            break

    rclpy.init(args=args)
    node = PathRecorder(run_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # We try to save on exit as a backup, but the Trigger Topic is the primary method now
        node.save_to_csv() 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()