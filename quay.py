import socket
import sys
import threading
import json
import time

ESP32_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.26"
CMD_PORT = 8889     # Cổng gửi lệnh xuống ESP32
TELEM_PORT = 8888   # Cổng nhận telemetry từ ESP32

# 1. Socket gửi lệnh
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 2. Socket nhận telemetry từ ESP32
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock_rx.bind(("0.0.0.0", TELEM_PORT))
except Exception as e:
    print(f"[!] Không thể mở cổng {TELEM_PORT}: {e}")
    print("    Kiểm tra xem có script nào khác đang chạy chiếm cổng 8888 không.")
    sys.exit(1)

# Biến lưu trữ telemetry mới nhất
latest_telem = {}
telem_lock = threading.Lock()
running = True

def rx_thread():
    """Luồng ngầm liên tục đón gói tin Telemetry từ ESP32."""
    global latest_telem, running
    sock_rx.settimeout(1.0)
    while running:
        try:
            data, _ = sock_rx.recvfrom(2048)
            t_data = json.loads(data.decode("utf-8", errors="ignore").strip())
            with telem_lock:
                latest_telem = t_data
        except (socket.timeout, json.JSONDecodeError):
            continue
        except Exception:
            break

t = threading.Thread(target=rx_thread, daemon=True)
t.start()

print("=" * 65)
print(f"[*] Đã kết nối UDP tới ESP32 ({ESP32_IP}:{CMD_PORT})")
print(f"[*] Đang đón Telemetry trên cổng {TELEM_PORT}...")
print("    - Lệnh đọc: odom, status, enc, imu, batt")
print("    - Lệnh điều khiển: enc reset, spin 1.0, stop, clear")
print("    - Gõ 'exit' để thoát.")
print("=" * 65 + "\n")

# Chờ 0.5s để đón gói đầu tiên
time.sleep(0.5)

try:
    while True:
        cmd = input("ESP32-WiFi > ").strip()
        if not cmd:
            continue
        if cmd.lower() in ("exit", "quit"):
            break

        # Đọc dữ liệu Telemetry mới nhất nếu người dùng gõ lệnh tra cứu
        with telem_lock:
            t_copy = latest_telem.copy()

        if cmd.lower() in ("odom", "status"):
            if not t_copy:
                print("[!] Chưa nhận được gói tin Telemetry nào từ ESP32.")
            else:
                th_deg = t_copy.get("th", 0.0) * 57.2957795
                print("-" * 55)
                print(f"  Chế độ (Mode) : {t_copy.get('md', 'N/A')}")
                print(f"  Tọa độ Odometry: x = {t_copy.get('x', 0):.4f} m | y = {t_copy.get('y', 0):.4f} m")
                print(f"  Góc Odom (th)  : {th_deg:+.2f}° ({t_copy.get('th', 0):.4f} rad)")
                print(f"  Góc IMU (yw)   : {t_copy.get('yw', 0):+.2f}°")
                print(f"  Vận tốc        : v = {t_copy.get('v', 0):.3f} m/s | w = {t_copy.get('w', 0):.3f} rad/s")
                print(f"  Số xung Encoder: CL = {t_copy.get('cl', 0)} | CR = {t_copy.get('cr', 0)}")
                print(f"  Điện áp pin    : {t_copy.get('vb', 0):.2f} V (Trạng thái lỗi: 0x{t_copy.get('st', 0):04X})")
                print("-" * 55)

        elif cmd.lower() == "enc":
            if not t_copy:
                print("[!] Chưa nhận được dữ liệu encoder.")
            else:
                print(f"  Encoder: CL = {t_copy.get('cl', 0)} xung | CR = {t_copy.get('cr', 0)} xung")

        elif cmd.lower() == "imu":
            if not t_copy:
                print("[!] Chưa nhận được dữ liệu IMU.")
            else:
                print(f"  IMU: Roll = {t_copy.get('rp', 0):.2f}° | Pitch = {t_copy.get('pp', 0):.2f}° | Yaw = {t_copy.get('yw', 0):.2f}°")
                print(f"  Gyro Z = {t_copy.get('gz', 0):.4f} rad/s | Acc Z = {t_copy.get('az', 0):.3f} g")

        elif cmd.lower() == "batt":
            if not t_copy:
                print("[!] Chưa nhận được thông số pin.")
            else:
                print(f"  Điện áp pin: {t_copy.get('vb', 0):.2f} V")

        # Gửi chuỗi lệnh xuống ESP32 để xử lý các lệnh hành động
        sock_tx.sendto((cmd + "\n").encode("utf-8"), (ESP32_IP, CMD_PORT))

except KeyboardInterrupt:
    pass
finally:
    running = False
    sock_tx.sendto(b"stop\n", (ESP32_IP, CMD_PORT))
    sock_rx.close()
    sock_tx.close()
    print("\n[*] Đã gửi lệnh STOP và đóng kết nối.")