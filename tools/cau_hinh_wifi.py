#!/usr/bin/env python3
"""
cau_hinh_wifi.py — Điền WIFI_SSID / WIFI_PASS / LAPTOP_IP vào config.h.

VÌ SAO CẦN SCRIPT NÀY
---------------------
Ba dòng đầu của config.h là chỗ dễ sai nhất khi bắt đầu đồ án:
    • Gõ nhầm IP laptop (ví dụ 192.168.1.10 thay vì .100)  → ESP32 gửi telemetry
      vào khoảng không, bạn ngồi chờ mãi không thấy dữ liệu.
    • Quên dấu ngoặc kép                             → lỗi biên dịch khó hiểu.
    • Mật khẩu WPA2 ngắn hơn 8 ký tự                 → ESP32 không kết nối được,
      nhưng thông báo lỗi rất mơ hồ.
Script này kiểm tra hết và tự sửa file, đồng thời sao lưu bản cũ.

CÁCH DÙNG
---------
    # 1. Xem giá trị hiện tại + IP của chính máy này
    python tools/cau_hinh_wifi.py

    # 2. Tự dò IP máy này rồi điền luôn (khuyến nghị)
    python tools/cau_hinh_wifi.py --ssid "TenWifiNha" --pass "matkhau123" --auto-ip

    # 3. Chỉ định IP thủ công
    python tools/cau_hinh_wifi.py --ssid "TenWifiNha" --pass "matkhau123" --laptop-ip 192.168.1.42

    # 4. Chỉ đổi SSID, giữ nguyên phần còn lại
    python tools/cau_hinh_wifi.py --ssid "TenWifiKhac"

LƯU Ý
-----
• SSID và mật khẩu WiFi có phân biệt CHỮ HOA/THƯỜNG.
• Nếu bạn dùng mạng 5 GHz: **ESP32 chỉ hỗ trợ 2.4 GHz**. Hãy phát thêm một SSID
  2.4 GHz hoặc bật chế độ mixed 2.4/5 GHz trên router.
• Sau khi nạp firmware, gõ `wifi ip` trên Serial Monitor để biết IP của ESP32 —
  bạn cần số đó để chạy ROS 2.
"""

import argparse
import os
import re
import shutil
import socket
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _console import setup_console          # noqa: E402
setup_console()

# Đường dẫn config.h tính từ vị trí script (tools/ -> ../firmware/esp32_base/include/)
GOC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_H = os.path.join(GOC, "firmware", "esp32_base", "include", "config.h")


def ip_cua_may_nay():
    """
    Dò địa chỉ IPv4 LAN của chính máy đang chạy script.

    Mẹo: mở socket UDP rồi `connect()` tới một địa chỉ ngoài. UDP `connect()`
    KHÔNG gửi gói tin nào — nó chỉ hỏi bảng định tuyến của hệ điều hành xem
    "nếu đi ra ngoài thì dùng card mạng nào", rồi đọc IP của card đó.
    Vì vậy hàm này chạy được cả khi không có Internet.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        # Không có mạng → thử lấy IP của hostname
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return None


def doc_hien_tai(text):
    """Trích giá trị hiện tại của 3 macro."""
    out = {}
    for key in ("WIFI_SSID", "WIFI_PASS", "LAPTOP_IP"):
        m = re.search(rf'^#define\s+{key}\s+"([^"]*)"', text, re.MULTILINE)
        out[key] = m.group(1) if m else None
    return out


def thay_the(text, key, value):
    """Thay giá trị của một macro, giữ nguyên phần căn lề và chú thích cuối dòng."""
    pattern = re.compile(rf'^(#define\s+{key}\s+)"[^"]*"', re.MULTILINE)

    def repl(m):
        return f'{m.group(1)}"{value}"'

    new_text, n = pattern.subn(repl, text)
    if n == 0:
        raise SystemExit(f"LỖI: không tìm thấy dòng `#define {key} \"...\"` trong config.h.\n"
                         f"     File có bị sửa tay làm mất dòng đó không?")
    return new_text


def kiem_tra_ip(ip):
    """Kiểm tra IPv4 hợp lệ và không phải địa chỉ đặc biệt."""
    if not re.fullmatch(r"\d{1,3}(\.\d{1,3}){3}", ip or ""):
        raise SystemExit(f"LỖI: '{ip}' không phải địa chỉ IPv4 hợp lệ.")
    oc = [int(x) for x in ip.split(".")]
    if any(x > 255 for x in oc):
        raise SystemExit(f"LỖI: '{ip}' có octet > 255.")
    if ip.startswith("127."):
        raise SystemExit("LỖI: 127.x.x.x là loopback, không dùng được.")
    if ip.startswith("169.254."):
        raise SystemExit("LỖI: 169.254.x.x là địa chỉ 'link-local' — nghĩa là máy bạn\n"
                         "     CHƯA lấy được IP từ router. Hãy kiểm tra WiFi trước.")
    return True


def main():
    ap = argparse.ArgumentParser(
        description="Điền WIFI_SSID / WIFI_PASS / LAPTOP_IP vào config.h. "
                    "Chạy không có tham số để XEM giá trị hiện tại.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ssid", help="tên WiFi (2.4 GHz!)")
    ap.add_argument("--pass", dest="matkhau", help="mật khẩu WiFi (>= 8 ký tự)")
    ap.add_argument("--laptop-ip", help="IP của máy tính chạy ROS 2, ví dụ 192.168.1.42")
    ap.add_argument("--auto-ip", action="store_true",
                    help="tự dò IP của máy này rồi dùng làm LAPTOP_IP")
    ap.add_argument("--config", default=CONFIG_H, help="đường dẫn tới config.h")
    args = ap.parse_args()

    path = args.config
    if not os.path.isfile(path):
        raise SystemExit(f"LỖI: không tìm thấy file:\n     {path}\n"
                         f"     Kiểm tra xem bạn có đang chạy script từ đúng thư mục dự án không.")

    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    cur = doc_hien_tai(text)
    ip_may = ip_cua_may_nay()

    # ---------------- Chế độ CHỈ XEM ----------------
    if not (args.ssid or args.matkhau or args.laptop_ip or args.auto_ip):
        print()
        print("=" * 72)
        print("  CẤU HÌNH WiFi HIỆN TẠI  —  firmware/esp32_base/include/config.h")
        print("=" * 72)
        print(f"  File        : {path}")
        print()
        print(f"  WIFI_SSID   : {cur['WIFI_SSID']}")
        print(f"  WIFI_PASS   : {'*' * len(cur['WIFI_PASS'] or '')}")
        print(f"  LAPTOP_IP   : {cur['LAPTOP_IP']}")
        print()
        print("-" * 72)
        print(f"  IP của MÁY NÀY (máy đang chạy script): {ip_may or 'không dò được'}")
        if ip_may and cur["LAPTOP_IP"] != ip_may:
            print(f"  ⚠️  LAPTOP_IP trong config.h ({cur['LAPTOP_IP']}) KHÁC IP máy này.")
            print(f"      Nếu bạn định chạy ROS 2 trên chính máy này, hãy sửa:")
            print(f"        python tools/cau_hinh_wifi.py --auto-ip")
        elif ip_may:
            print("  ✅ LAPTOP_IP trùng IP máy này — đúng.")
        print("-" * 72)
        print()
        print("  Để cấu hình, chạy:")
        print('    python tools/cau_hinh_wifi.py --ssid "TenWifiNha" \\')
        print('                                    --pass "matkhau123" --auto-ip')
        print()
        print("  ⚠️  ESP32 CHỈ hỗ trợ WiFi 2.4 GHz. Nếu router của bạn phát 5 GHz,")
        print("      hãy bật thêm SSID 2.4 GHz hoặc chế độ mixed 2.4/5 GHz.")
        print("  ⚠️  Sau khi nạp firmware, gõ `wifi ip` trên Serial Monitor để lấy")
        print("      IP của ESP32 — bạn cần số đó để chạy ROS 2.")
        print("=" * 72)
        print()
        return

    # ---------------- Kiểm tra dữ liệu nhập ----------------
    if args.ssid is not None:
        if not args.ssid:
            raise SystemExit("LỖI: SSID không được để trống.")
        if '"' in args.ssid:
            raise SystemExit("LỖI: SSID không được chứa dấu ngoặc kép.")

    if args.matkhau is not None and args.matkhau:
        if len(args.matkhau) < 8:
            print(f"⚠️  CẢNH BÁO: mật khẩu chỉ {len(args.matkhau)} ký tự. "
                  f"WPA2 yêu cầu >= 8 ký tự.")
            print("    ESP32 sẽ KHÔNG kết nối được. (Vẫn tiếp tục vì có thể bạn dùng WPA3/enterprise.)")

    laptop_ip = args.laptop_ip
    if args.auto_ip:
        if not ip_may:
            raise SystemExit("LỖI: không tự dò được IP của máy này.\n"
                             "     Hãy chạy `ipconfig` rồi truyền --laptop-ip thủ công.")
        laptop_ip = ip_may
    if laptop_ip:
        kiem_tra_ip(laptop_ip)
        if laptop_ip.endswith(".1"):
            print(f"⚠️  CẢNH BÁO: {laptop_ip} thường là địa chỉ ROUTER, không phải laptop.")
            print("    Hãy kiểm tra lại bằng `ipconfig`.")

    # ---------------- Áp dụng ----------------
    moi = text
    thay_doi = []
    if args.ssid is not None:
        moi = thay_the(moi, "WIFI_SSID", args.ssid)
        thay_doi.append(("WIFI_SSID", cur["WIFI_SSID"], args.ssid))
    if args.matkhau is not None:
        moi = thay_the(moi, "WIFI_PASS", args.matkhau)
        thay_doi.append(("WIFI_PASS", "*" * len(cur["WIFI_PASS"] or ""),
                         "*" * len(args.matkhau)))
    if laptop_ip:
        moi = thay_the(moi, "LAPTOP_IP", laptop_ip)
        thay_doi.append(("LAPTOP_IP", cur["LAPTOP_IP"], laptop_ip))

    if moi == text:
        print("\nKhông có gì thay đổi (giá trị mới giống giá trị cũ).\n")
        return

    # Sao lưu rồi ghi
    bak = path + ".bak"
    shutil.copy2(path, bak)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(moi)

    print()
    print("=" * 72)
    print("  ĐÃ CẬP NHẬT config.h")
    print("=" * 72)
    print(f"  {path}")
    print()
    for key, cu, m in thay_doi:
        print(f"  {key:<12} {cu}  →  {m}")
    print()
    print(f"  Bản cũ đã sao lưu tại: {os.path.basename(bak)}")
    print("-" * 72)
    print("  BƯỚC TIẾP THEO:")
    print("    1. cd firmware/esp32_base")
    print("    2. pio run -t upload")
    print("    3. pio device monitor -b 115200")
    print("    4. Trên Serial Monitor gõ:  wifi")
    print("       → ghi lại IP của ESP32 (bạn cần nó cho ROS 2)")
    print()
    print("  ⚠️  Nếu ESP32 in 'CHUA KET NOI' mãi:")
    print("      • Kiểm tra WiFi là 2.4 GHz (ESP32 không hỗ trợ 5 GHz)")
    print("      • Kiểm tra mật khẩu (phân biệt hoa/thường)")
    print("      • Kiểm tra router có bật 'AP isolation' không")
    print("=" * 72)
    print()


if __name__ == "__main__":
    main()
