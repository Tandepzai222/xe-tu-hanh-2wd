#!/usr/bin/env python3
"""
udp_test.py — Kiểm tra kết nối UDP giữa laptop và ESP32 TRƯỚC khi cài ROS 2.

Công dụng
---------
    ping   : đo RTT và tỉ lệ mất gói (chạy TRƯỚC khi cài ROS 2)
    mon    : chỉ in telemetry nhận được (kiểm tra luồng dữ liệu)
    drive  : gửi lệnh cmd_vel (v, w) trong N giây — thay thế tạm cho ROS 2
    square : cho robot chạy hình vuông (thay thế tạm cho Nav2)

Cách dùng
---------
    python tools/udp_test.py ping  --ip 192.168.1.50
    python tools/udp_test.py mon   --ip 192.168.1.50
    python tools/udp_test.py drive --ip 192.168.1.50 --v 0.15 --w 0.0 --secs 5
    python tools/udp_test.py square --ip 192.168.1.50 --side 0.5

⚠️  AN TOÀN: luôn đặt robot trên giá kê (bánh quay tự do) trong lần chạy đầu.
"""

import argparse
import json
import math
import socket
import statistics
import sys
import time

PORT_TELEM = 8888      # ESP32 → laptop
PORT_CMD = 8889        # laptop → ESP32


class Link:
    def __init__(self, ip, t_port=PORT_TELEM, c_port=PORT_CMD, timeout=1.0):
        self.ip = ip
        self.c_port = c_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", t_port))
        self.sock.settimeout(timeout)

    def send(self, obj):
        payload = (json.dumps(obj) + "\n").encode()
        self.sock.sendto(payload, (self.ip, self.c_port))

    def recv(self):
        data, _ = self.sock.recvfrom(2048)
        return json.loads(data.decode(errors="ignore").strip())

    def drain(self):
        """Bỏ hết gói đang tồn đọng trong buffer."""
        self.sock.settimeout(0.001)
        try:
            while True:
                self.sock.recvfrom(2048)
        except (socket.timeout, BlockingIOError):
            pass
        finally:
            self.sock.settimeout(1.0)


def cmd_ping(a):
    link = Link(a.ip, timeout=2.0)
    print(f"\nĐang ping {a.ip}:{PORT_CMD}  ({a.count} gói)...\n")
    rtts = []
    lost = 0

    for i in range(a.count):
        link.drain()
        t0 = time.perf_counter()
        link.send({"c": "ping", "seq": i, "ts": int(time.time() * 1000) % 100000})
        try:
            while True:
                m = link.recv()
                if m.get("c") == "pong":
                    rtt = (time.perf_counter() - t0) * 1000.0
                    rtts.append(rtt)
                    print(f"  #{i:3d}  RTT = {rtt:7.2f} ms   "
                          f"(ESP32 millis={m.get('te')})")
                    break
        except socket.timeout:
            lost += 1
            print(f"  #{i:3d}  ⏱  KHÔNG PHẢN HỒI")
        time.sleep(a.interval)

    print("\n" + "=" * 56)
    print(f"  Gửi      : {a.count}")
    print(f"  Nhận     : {len(rtts)}")
    print(f"  Mất      : {lost}  ({lost/a.count*100:.1f}%)")
    if rtts:
        print(f"  RTT min  : {min(rtts):.2f} ms")
        print(f"  RTT tb   : {statistics.mean(rtts):.2f} ms")
        print(f"  RTT max  : {max(rtts):.2f} ms")
        print(f"  Jitter   : {statistics.pstdev(rtts):.2f} ms  (độ lệch chuẩn)")
    print("=" * 56)
    if lost:
        print("  ⚠️  Có mất gói → kiểm tra WiFi (RSSI), hoặc đặt ESP32 gần router hơn.")
    if rtts and statistics.mean(rtts) > 30:
        print("  ⚠️  RTT trung bình > 30 ms → WiFi yếu. Nav2 cần RTT < 20 ms để ổn định.")
    print()


def cmd_mon(a):
    link = Link(a.ip, timeout=a.secs + 2)
    print(f"\nĐang lắng nghe telemetry trên cổng {PORT_TELEM}... (Ctrl+C để dừng)\n")
    print(f"{'t_ms':>8} {'mode':<8} {'x':>8} {'y':>8} {'yaw°':>8} "
          f"{'v':>7} {'w':>7} {'vL':>7} {'vR':>7} {'VB':>6} {'st':>5} {'hz':>6}")
    print("-" * 100)

    t_end = time.time() + a.secs
    n = 0
    try:
        while time.time() < t_end:
            m = link.recv()
            n += 1
            print(f"{m.get('t',0):>8} {m.get('md',''):<8} "
                  f"{m.get('x',0):>8.3f} {m.get('y',0):>8.3f} {m.get('yw',0):>8.2f} "
                  f"{m.get('v',0):>7.3f} {m.get('w',0):>7.3f} "
                  f"{m.get('vl',0):>7.3f} {m.get('vr',0):>7.3f} "
                  f"{m.get('vb',0):>6.2f} {m.get('st',0):>5} {m.get('hz',0):>6.1f}")
    except socket.timeout:
        print("\n⏱  Hết thời gian chờ gói tin.")
    except KeyboardInterrupt:
        pass

    print("-" * 100)
    if n == 0:
        print("❌ Không nhận được gói nào!")
        print("   Kiểm tra: (1) LAPTOP_IP trong config.h, (2) cùng mạng WiFi,")
        print("   (3) firewall Windows có chặn UDP 8888 không.")
    else:
        print(f"✅ Nhận {n} gói trong {a.secs:.0f} s → {n/a.secs:.1f} Hz "
              f"(kỳ vọng {1000/20:.0f} Hz với TELEM_PERIOD_MS=20)")
    print()


def cmd_drive(a):
    link = Link(a.ip, timeout=0.05)
    print(f"\n🚗 Gửi lệnh: v={a.v} m/s, w={a.w} rad/s trong {a.secs} s")
    print("   (Ctrl+C để dừng khẩn cấp)\n")

    t_end = time.time() + a.secs
    n_tx = n_rx = 0
    try:
        while time.time() < t_end:
            link.send({"c": "vel", "v": a.v, "w": a.w})
            n_tx += 1
            try:
                m = link.recv()
                n_rx += 1
                if n_tx % 5 == 0:
                    print(f"  t={m.get('t',0)/1000:6.2f}s  x={m.get('x',0):+7.3f} "
                          f"y={m.get('y',0):+7.3f}  yaw={m.get('yw',0):+7.2f}°  "
                          f"v={m.get('v',0):+6.3f}  w={m.get('w',0):+6.3f}")
            except socket.timeout:
                pass
            time.sleep(1.0 / 20.0)
    except KeyboardInterrupt:
        print("\n⛔ Dừng theo yêu cầu.")
    finally:
        # ⚠️ LUÔN gửi lệnh dừng khi thoát
        for _ in range(5):
            link.send({"c": "stop"})
            time.sleep(0.02)
        print(f"\n  Đã gửi {n_tx} lệnh, nhận {n_rx} gói telemetry. Đã gửi lệnh STOP.")
    print()


def cmd_square(a):
    """Chạy hình vuông bằng vòng điều khiển mù (dead reckoning) — test tổng thể."""
    link = Link(a.ip, timeout=0.05)
    print(f"\n⬛ Chạy hình vuông cạnh {a.side} m, v={a.v} m/s\n")

    def go(v, w, secs, note):
        print(f"  → {note} ({secs:.2f} s)")
        t_end = time.time() + secs
        while time.time() < t_end:
            link.send({"c": "vel", "v": v, "w": w})
            try:
                link.recv()
            except socket.timeout:
                pass
            time.sleep(1.0 / 20.0)

    try:
        for i in range(4):
            go(a.v, 0.0, a.side / a.v, f"Cạnh {i+1}")
            go(0.0, 0.0, 0.3, "Ổn định")
            go(0.0, a.w, (math.pi / 2) / a.w, f"Quay 90° #{i+1}")
            go(0.0, 0.0, 0.3, "Ổn định")
    except KeyboardInterrupt:
        print("\n⛔ Dừng theo yêu cầu.")
    finally:
        for _ in range(5):
            link.send({"c": "stop"})
            time.sleep(0.02)
        print("\n  Đã gửi lệnh STOP.\n")


def main():
    ap = argparse.ArgumentParser(description="Kiểm tra UDP ESP32 ↔ laptop (docs/05 §2)")
    ap.add_argument("mode", choices=["ping", "mon", "drive", "square"])
    ap.add_argument("--ip", required=True, help="IP của ESP32 (xem lệnh `wifi ip`)")
    ap.add_argument("--count", type=int, default=50, help="[ping] số gói")
    ap.add_argument("--interval", type=float, default=0.05, help="[ping] giãn cách (s)")
    ap.add_argument("--secs", type=float, default=10.0, help="[mon|drive] thời lượng")
    ap.add_argument("--v", type=float, default=0.15, help="[drive|square] vận tốc (m/s)")
    ap.add_argument("--w", type=float, default=0.0, help="[drive] vận tốc góc (rad/s)")
    ap.add_argument("--side", type=float, default=0.5, help="[square] cạnh (m)")
    args = ap.parse_args()

    if args.mode == "ping":
        cmd_ping(args)
    elif args.mode == "mon":
        cmd_mon(args)
    elif args.mode == "drive":
        cmd_drive(args)
    else:
        args.w = 1.0
        cmd_square(args)


if __name__ == "__main__":
    main()
