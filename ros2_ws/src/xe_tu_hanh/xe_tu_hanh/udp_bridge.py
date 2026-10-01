#!/usr/bin/env python3
"""
udp_bridge.py — Cầu nối ROS 2 <-> ESP32 qua WiFi UDP.

Kiến trúc
---------
    ESP32  --(UDP 8888, JSON 50 Hz)-->  udp_bridge  --(/odom, /imu/data, TF)-->  ROS 2
    ESP32  <--(UDP 8889, JSON 20 Hz)--  udp_bridge  <--(/cmd_vel)---------------  Nav2

Đây là GIAI ĐOẠN 1 của thiết kế (docs/05 §1.2): cầu nối UDP JSON tự viết.
Ưu điểm: debug được bằng `nc`/Python, không phụ thuộc phiên bản ROS, thấy rõ
từng byte. Giai đoạn 2 (tuỳ chọn) là chuyển sang micro-ROS.

⚠️ QUY TẮC TF — chỉ được có MỘT nguồn phát cho mỗi cạnh TF:
    • Nếu chạy EKF (robot_localization) → EKF phát `odom -> base_footprint`
      → đặt tham số `publish_tf:=false` cho node này.
    • Nếu KHÔNG chạy EKF → node này phát `odom -> base_footprint`
      → đặt `publish_tf:=true`.
   Phát cả hai sẽ làm TF bị giật/nhảy loạn.

Chạy thử
--------
    ros2 run xe_tu_hanh udp_bridge --ros-args -p esp32_ip:=192.168.1.50
"""

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


# ============================================================================
#  Ma trận hiệp phương sai — docs/02 §5
#  Đây KHÔNG phải con số trang trí: EKF/Nav2 dùng chúng để quyết định tin ai.
# ============================================================================

# Odometry bánh xe: x, y trôi VÔ HẠN (tích phân) → hiệp phương sai rất lớn
# để EKF biết "đừng tin vị trí tuyệt đối của odometry".
ODOM_POSE_COV = [
    0.02, 0.0,  0.0,  0.0,  0.0,  0.0,     # x  [m²]
    0.0,  0.02, 0.0,  0.0,  0.0,  0.0,     # y
    0.0,  0.0,  1e6,  0.0,  0.0,  0.0,     # z  (2D → gần như không dùng)
    0.0,  0.0,  0.0,  1e6,  0.0,  0.0,     # roll
    0.0,  0.0,  0.0,  0.0,  1e6,  0.0,     # pitch
    0.0,  0.0,  0.0,  0.0,  0.0,  0.03,    # yaw [rad²] — tin được ở ngắn hạn
]

# Vận tốc: đây là thứ odometry ĐÁNG TIN NHẤT → hiệp phương sai nhỏ.
ODOM_TWIST_COV = [
    0.0025, 0.0,    0.0,    0.0,    0.0,    0.0,    # vx [m/s]²
    0.0,    0.0025, 0.0,    0.0,    0.0,    0.0,    # vy (luôn ~0, robot vi sai)
    0.0,    0.0,    1e6,    0.0,    0.0,    0.0,    # vz
    0.0,    0.0,    0.0,    1e6,    0.0,    0.0,    # wx
    0.0,    0.0,    0.0,    0.0,    1e6,    0.0,    # wy
    0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   # wz [rad/s]²
]

# IMU: roll/pitch ổn (từ gia tốc kế + lọc bù), yaw KÉM (tích phân gyro → trôi).
IMU_ORIENT_COV = [
    0.01, 0.0,  0.0,
    0.0,  0.01, 0.0,
    0.0,  0.0,  0.08,      # yaw — để lớn hơn để EKF không tin tuyệt đối
]
IMU_GYRO_COV = [
    0.0004, 0.0,    0.0,
    0.0,    0.0004, 0.0,
    0.0,    0.0,    0.0004,
]
IMU_ACCEL_COV = [
    0.04, 0.0,  0.0,
    0.0,  0.04, 0.0,
    0.0,  0.0,  0.04,
]

# Ngưỡng bit trạng thái — PHẢI khớp CHÍNH XÁC với safety.h
# ⚠️ Nếu bạn sửa safety.h mà quên sửa ở đây, ROS 2 sẽ báo sai loại lỗi —
#    ví dụ báo "hết pin" khi thực ra robot đang bị kẹt bánh.
ST_STALL_L     = 0x0001
ST_STALL_R     = 0x0002
ST_BATT_LOW    = 0x0004
ST_ESTOP       = 0x0008
ST_IMU_FAIL    = 0x0010
ST_CMD_TIMEOUT = 0x0020
ST_WIFI_DOWN   = 0x0040
ST_WARN        = 0x0080

STATUS_NAMES = {
    ST_STALL_L:     "STALL_L",
    ST_STALL_R:     "STALL_R",
    ST_BATT_LOW:    "BATT_LOW",
    ST_ESTOP:       "ESTOP",
    ST_IMU_FAIL:    "IMU_FAIL",
    ST_CMD_TIMEOUT: "CMD_TIMEOUT",
    ST_WIFI_DOWN:   "WIFI_DOWN",
    ST_WARN:        "WARN",
}


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Chuyển yaw (rad) sang quaternion. Robot phẳng → roll = pitch = 0."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Chuyển 3 góc Euler (rad, thứ tự ZYX) sang quaternion."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


class UdpBridge(Node):

    def __init__(self):
        super().__init__("udp_bridge")

        # ------------------------- Tham số -------------------------
        self.declare_parameter("esp32_ip", "192.168.1.50")
        self.declare_parameter("port_telemetry", 8888)     # ESP32 -> laptop
        self.declare_parameter("port_command", 8889)       # laptop -> ESP32
        self.declare_parameter("publish_tf", False)        # xem cảnh báo ở đầu file
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("cmd_rate_hz", 20.0)        # tần số gửi lệnh
        self.declare_parameter("cmd_timeout_s", 0.5)       # không có cmd_vel -> gửi 0
        self.declare_parameter("telemetry_timeout_s", 2.0) # mất telemetry -> cảnh báo

        self.ip = str(self.get_parameter("esp32_ip").value)
        self.port_t = int(self.get_parameter("port_telemetry").value)
        self.port_c = int(self.get_parameter("port_command").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.telem_timeout = float(self.get_parameter("telemetry_timeout_s").value)

        # ------------------------- Socket UDP -------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port_t))
        self.sock.setblocking(False)
        self.dest = (self.ip, self.port_c)

        # ------------------------- QoS -------------------------
        # Sensor data: BEST_EFFORT + KEEP_LAST(1) → luôn dùng mẫu mới nhất,
        # không tích gói cũ. Đây là chuẩn cho dữ liệu cảm biến tần số cao.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # cmd_vel: RELIABLE, depth 1 — chỉ quan tâm lệnh mới nhất
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------- Publisher -------------------------
        self.pub_odom = self.create_publisher(Odometry, "odom", sensor_qos)
        self.pub_imu = self.create_publisher(Imu, "imu/data", sensor_qos)
        self.pub_batt = self.create_publisher(BatteryState, "battery_state", sensor_qos)
        self.pub_status = self.create_publisher(String, "esp32/status", cmd_qos)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info("publish_tf = TRUE  -> node nay phat odom->base_footprint")
        else:
            self.tf_broadcaster = None
            self.get_logger().info(
                "publish_tf = FALSE -> khong phat TF (robot_localization EKF se phat)")

        # ------------------------- Subscriber -------------------------
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, cmd_qos)

        # ------------------------- Service -------------------------
        self.create_service(Empty, "reset_odometry", self.on_reset_odom)

        # ------------------------- Trạng thái -------------------------
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = 0.0
        self.last_rx_time = 0.0
        self.last_seq = -1
        self.n_rx = 0
        self.n_lost = 0
        self.n_tx = 0
        self.last_rate_calc = time.time()
        self.rx_rate = 0.0
        self.warned_no_telem = False

        # ------------------------- Timer -------------------------
        # Vòng nhận: quét socket ở 250 Hz (không chặn) để không bỏ sót gói.
        self.create_timer(0.004, self.poll_socket)
        # Vòng gửi: gửi cmd_vel ở 20 Hz như Nav2 mong đợi.
        self.create_timer(1.0 / self.cmd_rate, self.send_cmd)
        # Thống kê mỗi 5 s.
        self.create_timer(5.0, self.report_stats)

        self.get_logger().info(
            f"udp_bridge san sang | nhan telemetry cong {self.port_t} | "
            f"gui lenh toi {self.ip}:{self.port_c} @ {self.cmd_rate:.0f} Hz")

    # ==================================================================
    #  Nhận telemetry
    # ==================================================================
    def poll_socket(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            except OSError as e:
                self.get_logger().error(f"Loi socket: {e}")
                return

            try:
                m = json.loads(data.decode("utf-8", errors="ignore").strip())
            except json.JSONDecodeError:
                self.get_logger().warn("Bo qua goi JSON khong hop le", throttle_duration_sec=5.0)
                continue

            if m.get("c") == "pong":
                self.get_logger().debug(f"pong: {m}")
                continue

            self.handle_telemetry(m)

    def handle_telemetry(self, m):
        now = self.get_clock().now().to_msg()
        self.last_rx_time = time.time()
        self.n_rx += 1

        # ---- Phát hiện mất gói qua số thứ tự ----
        seq = m.get("seq")
        if isinstance(seq, int):
            if self.last_seq >= 0 and seq > self.last_seq + 1:
                self.n_lost += seq - self.last_seq - 1
            self.last_seq = seq

        # ---- Trạng thái lỗi ----
        st = int(m.get("st", 0))
        if st:
            names = [n for bit, n in STATUS_NAMES.items() if st & bit]
            self.pub_status.publish(String(data=json.dumps({
                "status": st, "errors": names, "mode": m.get("md", "?"),
                "vbat": m.get("vb", 0.0),
            })))
            # Cảnh báo các lỗi nghiêm trọng (chỉ log khi bit đổi, xem _prev_st)
            if st != getattr(self, "_prev_st", 0):
                for n in names:
                    if n in ("ESTOP", "STALL_L", "STALL_R", "BATT_CRIT", "IMU_FAIL"):
                        self.get_logger().error(f"⚠️  ESP32 bao loi: {n}")
                    else:
                        self.get_logger().warn(f"ESP32 canh bao: {n}")
        self._prev_st = st

        # ---- /odom ----
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame

        od.pose.pose.position.x = float(m.get("x", 0.0))
        od.pose.pose.position.y = float(m.get("y", 0.0))
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation = yaw_to_quaternion(float(m.get("th", 0.0)))
        od.pose.covariance = ODOM_POSE_COV

        od.twist.twist.linear.x = float(m.get("v", 0.0))
        od.twist.twist.linear.y = 0.0
        od.twist.twist.angular.z = float(m.get("w", 0.0))
        od.twist.covariance = ODOM_TWIST_COV
        self.pub_odom.publish(od)

        # ---- /imu/data ----
        im = Imu()
        im.header.stamp = now
        im.header.frame_id = self.imu_frame

        # Gia tốc kế: ESP32 gửi theo đơn vị g → đổi sang m/s²
        G = 9.80665
        im.linear_acceleration.x = float(m.get("ax", 0.0)) * G
        im.linear_acceleration.y = float(m.get("ay", 0.0)) * G
        im.linear_acceleration.z = float(m.get("az", 0.0)) * G
        im.linear_acceleration_covariance = IMU_ACCEL_COV

        im.angular_velocity.x = float(m.get("gx", 0.0))
        im.angular_velocity.y = float(m.get("gy", 0.0))
        im.angular_velocity.z = float(m.get("gz", 0.0))
        im.angular_velocity_covariance = IMU_GYRO_COV

        roll = math.radians(float(m.get("rp", 0.0)))
        pitch = math.radians(float(m.get("pp", 0.0)))
        yaw = math.radians(float(m.get("yw", 0.0)))
        im.orientation = euler_to_quaternion(roll, pitch, yaw)
        im.orientation_covariance = IMU_ORIENT_COV
        self.pub_imu.publish(im)

        # ---- /battery_state ----
        bt = BatteryState()
        bt.header.stamp = now
        bt.header.frame_id = self.base_frame
        bt.voltage = float(m.get("vb", 0.0))
        # 3S LiPo: 12.6 V đầy, 9.9 V cạn (3.3 V/cell)
        bt.percentage = max(0.0, min(1.0, (bt.voltage - 9.9) / (12.6 - 9.9)))
        bt.present = True
        if bt.voltage < 10.5:
            bt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            bt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.pub_batt.publish(bt)

        # ---- TF odom -> base_footprint (CHỈ khi không dùng EKF) ----
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = od.pose.pose.position.x
            t.transform.translation.y = od.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = od.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # ---- Cảnh báo mất telemetry (chỉ một lần) ----
        self.warned_no_telem = False

    # ==================================================================
    #  Gửi lệnh điều khiển
    # ==================================================================
    def on_cmd_vel(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = time.time()

    def send_cmd(self):
        # Watchdog: Nav2 ngừng gửi cmd_vel → robot phải DỪNG, không được chạy tiếp.
        if (time.time() - self.last_cmd_time) > self.cmd_timeout:
            v = w = 0.0
        else:
            v, w = self.cmd_v, self.cmd_w

        payload = json.dumps({"c": "vel", "v": round(v, 4), "w": round(w, 4)}).encode()
        try:
            self.sock.sendto(payload, self.dest)
            self.n_tx += 1
        except OSError as e:
            self.get_logger().error(f"Khong gui duoc lenh: {e}", throttle_duration_sec=5.0)

    # ==================================================================
    #  Service reset odometry
    # ==================================================================
    def on_reset_odom(self, request, response):
        try:
            self.sock.sendto(json.dumps({"c": "reset_odom"}).encode(), self.dest)
            self.get_logger().info("Da gui lenh reset odometry toi ESP32")
        except OSError as e:
            self.get_logger().error(f"Khong gui duoc lenh reset: {e}")
        return response

    # ==================================================================
    #  Thống kê
    # ==================================================================
    def report_stats(self):
        now = time.time()
        dt = now - self.last_rate_calc
        self.last_rate_calc = now

        if self.n_rx > 0:
            self.rx_rate = self.n_rx / dt if dt > 0 else 0.0

        if (now - self.last_rx_time) > self.telem_timeout:
            if not self.warned_no_telem:
                self.get_logger().error(
                    f"❌ Khong nhan duoc telemetry tu {self.ip}:{self.port_t} "
                    f"trong {self.telem_timeout:.0f} s! Kiem tra WiFi / firewall / IP.")
                self.warned_no_telem = True
            self.n_rx = self.n_lost = self.n_tx = 0
            return

        total = self.n_rx + self.n_lost
        loss = (self.n_lost / total * 100.0) if total > 0 else 0.0
        self.get_logger().info(
            f"Telemetry {self.rx_rate:5.1f} Hz | mat goi {loss:5.2f}% "
            f"({self.n_lost}/{total}) | da gui {self.n_tx} lenh")
        self.n_rx = self.n_lost = self.n_tx = 0

    def destroy_node(self):
        # ⚠️ Gửi lệnh dừng trước khi thoát — nếu không robot sẽ chạy mãi
        #    cho tới khi watchdog 300 ms của ESP32 tự cắt.
        try:
            for _ in range(5):
                self.sock.sendto(json.dumps({"c": "stop"}).encode(), self.dest)
                time.sleep(0.02)
            self.get_logger().info("Da gui lenh STOP truoc khi thoat.")
        except OSError:
            pass
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
