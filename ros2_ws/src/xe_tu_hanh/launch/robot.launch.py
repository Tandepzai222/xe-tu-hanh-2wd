#!/usr/bin/env python3
"""
robot.launch.py — Khởi động toàn bộ chuỗi TF và cầu nối ESP32.

Bật lên:
    robot_state_publisher   (URDF -> TF các link tĩnh/động)
    joint_state_publisher   (phát góc bánh = 0, đủ để RViz hiển thị)
    ekf_filter_node         (robot_localization, tuỳ chọn — mặc định BẬT)
    udp_bridge              (giao tiếp ESP32)
    rviz2                   (tuỳ chọn)

Chạy:
    ros2 launch xe_tu_hanh robot.launch.py esp32_ip:=192.168.1.50
    ros2 launch xe_tu_hanh robot.launch.py esp32_ip:=192.168.1.50 use_rviz:=true

⚠️ VỀ TF: script này tự động đảm bảo CHỈ MỘT nguồn phát `odom -> base_footprint`:
       use_ekf:=true  → EKF phát, bridge KHÔNG phát
       use_ekf:=false → bridge phát, không có EKF
   Bạn KHÔNG cần (và không nên) chỉnh tay publish_tf.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("xe_tu_hanh")

    urdf_file    = os.path.join(pkg, "urdf", "xe_tu_hanh.urdf.xacro")
    ekf_params   = os.path.join(pkg, "config", "ekf.yaml")

    esp32_ip     = LaunchConfiguration("esp32_ip")
    use_ekf      = LaunchConfiguration("use_ekf")
    use_rviz     = LaunchConfiguration("use_rviz")
    use_jsp      = LaunchConfiguration("use_joint_state_publisher")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ------------------------------------------------------------------
    # TF: nếu dùng EKF thì bridge KHÔNG được phát TF (và ngược lại).
    # Đây là ràng buộc CỨNG của TF — vi phạm sẽ làm robot mất định vị.
    # ------------------------------------------------------------------
    bridge_publish_tf = PythonExpression(
        ["'false' if '", use_ekf, "' == 'true' else 'true'"]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file]),
            "use_sim_time": use_sim_time,
            "publish_frequency": 20.0,
        }],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_jsp),
        parameters=[{"use_sim_time": use_sim_time, "rate": 20}],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", "/odom")],
    )

    udp_bridge = Node(
        package="xe_tu_hanh",
        executable="udp_bridge",
        name="udp_bridge",
        output="screen",
        parameters=[{
            "esp32_ip": esp32_ip,
            "publish_tf": bridge_publish_tf,
            "use_sim_time": use_sim_time,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [robot_state_publisher, joint_state_publisher,
            ekf_node, udp_bridge, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "esp32_ip", default_value="192.168.1.50",
            description="IP cua ESP32 (xem bang lenh `wifi ip` tren Serial)"),
        DeclareLaunchArgument(
            "use_ekf", default_value="true",
            description="Dung robot_localization EKF (khuyen nghi: true)"),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="Mo RViz2"),
        DeclareLaunchArgument(
            "use_joint_state_publisher", default_value="true",
            description="Phat joint_states = 0 de RViz hien thi banh"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
