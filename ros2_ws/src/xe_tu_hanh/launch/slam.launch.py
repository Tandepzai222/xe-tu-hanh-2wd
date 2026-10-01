#!/usr/bin/env python3
"""
slam.launch.py — Chạy SLAM 2D (slam_toolbox, chế độ online asynchronous).

ĐIỀU KIỆN TIÊN QUYẾT (kiểm tra trước khi chạy):
    1. `robot.launch.py` đã chạy  → có TF odom -> base_footprint
    2. LiDAR đã chạy              → có topic /scan
    3. Odometry đã hiệu chuẩn     → sai số < 2% (docs/07 GĐ 3)

Chạy:
    ros2 launch xe_tu_hanh slam.launch.py
    ros2 launch xe_tu_hanh slam.launch.py use_rviz:=true

Lưu bản đồ sau khi quét xong:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/nha

Kiểm tra TF (nên chạy song song):
    ros2 run tf2_tools view_frames.py && evince frames.pdf
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("xe_tu_hanh")
    params_file = os.path.join(pkg, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"
            ])
        ]),
        launch_arguments={
            "slam_params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        slam,
        rviz,
    ])
