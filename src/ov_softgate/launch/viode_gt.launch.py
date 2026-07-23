"""
GT-mode launch: uses VIODE ground-truth segmentation colours for masking.

Usage:
  ros2 launch ov_softgate viode_gt.launch.py \\
      calib_file:=/path/to/kalibr_imucam_chain.yaml \\
      output_dir:=/path/to/results \\
      run_id:=1 \\
      mask_source:=gt

Parameters
----------
calib_file   Absolute path to kalibr_imucam_chain.yaml (Kalibr stereo calibration).
output_dir   Directory where CSV results are written.  Created if absent.
run_id       Integer suffix appended to output filenames (e.g. vio_path_run_1.csv).
mask_source  'gt' (default) — use VIODE segmentation colours.
             'yolo' — use /cam0/objects label maps from yolo_masker instead.
force_empty  Set true to disable masking (unmasked control experiment).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calib_file',  default_value='',
                              description='Absolute path to kalibr_imucam_chain.yaml'),
        DeclareLaunchArgument('output_dir',  default_value=os.path.expanduser('~/ov_results'),
                              description='Directory for CSV output files'),
        DeclareLaunchArgument('run_id',      default_value='0',
                              description='Run index appended to output filenames'),
        DeclareLaunchArgument('mask_source', default_value='gt',
                              description="'gt' or 'yolo'"),
        DeclareLaunchArgument('force_empty', default_value='false',
                              description='Publish all-zeros mask (unmasked control)'),

        Node(
            package='ov_softgate',
            executable='masker',
            name='semantic_masker',
            parameters=[{
                'force_empty': LaunchConfiguration('force_empty'),
                'dilation_kernel': 13,
                'max_mask_fraction': 0.80,
            }],
        ),

        Node(
            package='ov_softgate',
            executable='hybrid_speed_estimator',
            name='hybrid_speed_estimator',
            arguments=[LaunchConfiguration('run_id')],
            parameters=[{
                'calib_file':  LaunchConfiguration('calib_file'),
                'output_dir':  LaunchConfiguration('output_dir'),
                'mask_source': LaunchConfiguration('mask_source'),
                'min_features': 8,
                'min_disparity': 1.2,
                'orb_nfeatures': 100,
            }],
        ),

        Node(
            package='ov_softgate',
            executable='path_recorder',
            name='path_recorder',
            arguments=[LaunchConfiguration('run_id')],
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
