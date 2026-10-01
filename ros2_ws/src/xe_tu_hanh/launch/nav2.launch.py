#!/usr/bin/env python3
"""
nav2.launch.py — Chạy Nav2 để robot tự hành theo bản đồ đã lưu.

ĐIỀU KIỆN TIÊN QUYẾT:
    1. `robot.launch.py` đang chạy (TF + udp_bridge)
    2. LiDAR đang chạy (/scan)
    3. ĐÃ CÓ bản đồ .yaml/.pgm (từ `map_saver_cli`)
    4. ⚠️ KHÔNG chạy đồng thời `slam.launch.py` — cả hai đều phát
       `map -> odom` và sẽ tranh nhau làm hỏng định vị.

Chạy:
    ros2 launch xe_tu_hanh nav2.launch.py map:=$HOME/maps/nha.yaml
    ros2 launch xe_tu_hanh nav2.launch.py map:=$HOME/maps/nha.yaml use_rviz:=true

Đặt mục tiêu (chọn 1 trong 3 cách):
    # 1. Trong RViz2: bấm "2D Goal Pose" rồi click lên bản đồ
    # 2. Dòng lệnh:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, \\
       orientation: {w: 1.0}}}}"
    # 3. Python:
    python3 -c "..."   # xem docs/05 §6.4
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("xe_tu_hanh")
    params_file = os.path.join(pkg, "config", "nav2_params.yaml")

    map_yaml     = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz     = LaunchConfiguration("use_rviz")
    autostart    = LaunchConfiguration("autostart")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"
            ])
        ]),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "slam": "False",          # ⚠️ dùng AMCL + bản đồ, KHÔNG dùng SLAM
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=os.path.expanduser("~/maps/nha.yaml"),
            description="Duong dan file ban do .yaml da luu"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        nav2,
        rviz,
    ])
