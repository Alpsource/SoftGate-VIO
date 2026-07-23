"""
YOLO masker launch: runs yolo_masker with stereo cam0/cam1 image topics.

Usage:
  ros2 launch yolo_masker yolo_masker.launch.py \\
      model_path:=/path/to/yolo11n-seg.pt \\
      force_empty:=false

Parameters
----------
model_path            Absolute path to a YOLOv8/v11 segmentation .pt model.
                      Required — node logs an error and skips loading if empty.
force_empty           Publish all-zeros VIO mask (unmasked control experiment).
                      Object label maps (/cam0/objects) are still published.
use_flow_classifier   Enable ego-motion-compensated static/dynamic discrimination.
confidence_threshold  YOLO detection confidence cutoff.
dilation_kernel       Mask dilation in pixels (safety margin around detections).
max_mask_fraction     If masked fraction exceeds this, publish empty mask (guard).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_path',            default_value='',
                              description='Path to YOLO segmentation .pt model (required)'),
        DeclareLaunchArgument('force_empty',           default_value='false',
                              description='Publish all-zeros VIO mask (unmasked control)'),
        DeclareLaunchArgument('use_flow_classifier',   default_value='true',
                              description='Ego-motion-compensated static/dynamic discrimination'),
        DeclareLaunchArgument('confidence_threshold',  default_value='0.25',
                              description='YOLO detection confidence cutoff'),
        DeclareLaunchArgument('dilation_kernel',       default_value='13',
                              description='Mask dilation size in pixels'),
        DeclareLaunchArgument('max_mask_fraction',     default_value='0.80',
                              description='Coverage guard threshold'),

        Node(
            package='yolo_masker',
            executable='yolo_masker',
            name='yolo_masker',
            parameters=[{
                'model_path':           LaunchConfiguration('model_path'),
                'force_empty':          LaunchConfiguration('force_empty'),
                'use_flow_classifier':  LaunchConfiguration('use_flow_classifier'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'dilation_kernel':      LaunchConfiguration('dilation_kernel'),
                'max_mask_fraction':    LaunchConfiguration('max_mask_fraction'),
            }],
        ),
    ])
