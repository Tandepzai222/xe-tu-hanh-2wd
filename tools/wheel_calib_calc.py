#!/usr/bin/env python3
"""
wheel_calib_calc.py — Tính lại các tham số hiệu chuẩn từ số liệu đo thực tế.

Ba chế độ:
  1) rev   — bạn quay tay bánh đúng N vòng, đếm tổng xung       → ra N_enc
  2) roll  — bạn cho robot chạy quãng đường S đã biết           → ra d_pc trực tiếp
  3) spin  — bạn cho robot quay tại chỗ N vòng                  → ra L (wheel track)

Cách dùng
---------
    # 1. Quay tay 10 vòng, tổng xung đọc được là 19758
    python tools/wheel_calib_calc.py rev --counts 19758 --revolutions 10

    # 2. Chạy 3.000 m, odometry báo 2.845 m (hoặc tổng xung 27530)
    python tools/wheel_calib_calc.py roll --s-true 3.000 --counts 27530

    # 3. Quay tại chỗ 10 vòng: odom nói 3540°, IMU nói 3612°
    python tools/wheel_calib_calc.py spin --odom-deg 3540 --imu-deg 3612 --l-current 0.185
"""

import argparse
import math
import os
import sys

# ⚠️ Phải chạy TRƯỚC mọi print/argparse: Windows mặc định dùng cp1252 nên
#    tiếng Việt sẽ gây UnicodeEncodeError. Xem tools/_console.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _console import setup_console          # noqa: E402
setup_console()


def huong_dan():
    """In hướng dẫn khi chạy script không có tham số."""
    print()
    print("=" * 74)
    print("  wheel_calib_calc.py — TÍNH THAM SỐ HIỆU CHUẨN TỪ SỐ LIỆU ĐO")
    print("=" * 74)
    print()
    print("  Script này KHÔNG tự đo — nó TÍNH TOÁN từ số bạn đo được.")
    print("  Bạn phải chọn 1 trong 3 chế độ dưới đây:")
    print()
    print("-" * 74)
    print("  [1] rev  — quay tay bánh đúng N vòng, đếm tổng xung  →  ra N")
    print("-" * 74)
    print("      Dùng khi: bạn đã đánh dấu bánh, quay tay đúng vài vòng,")
    print("      rồi đọc tổng xung bằng lệnh `enc` trên Serial Monitor.")
    print()
    print("      Ví dụ (chính là phép đo của bạn — quay 10 vòng được 19758 xung):")
    print()
    print("        python tools/wheel_calib_calc.py rev --counts 19758 --revolutions 10")
    print()
    print("-" * 74)
    print("  [2] roll — cho robot chạy quãng đường đã biết  →  ra d_pc")
    print("-" * 74)
    print("      Dùng khi: bạn đẩy/chạy robot đúng 3.000 m rồi đọc tổng xung.")
    print("      Đây là cách CHÍNH XÁC NHẤT vì nó tính cả bán kính lăn của lốp.")
    print()
    print("      Ví dụ (chạy 3.000 m, tổng xung trung bình 2 bánh = 29020):")
    print()
    print("        python tools/wheel_calib_calc.py roll --s-true 3.000 --counts 29020")
    print()
    print("-" * 74)
    print("  [3] spin — quay tại chỗ N vòng  →  ra L (khoảng cách 2 bánh)")
    print("-" * 74)
    print("      Dùng khi: bạn gõ `cal spin 5` trên firmware, rồi đọc 2 con số")
    print("      'Goc theo ODOMETRY' và 'Goc theo IMU' mà nó in ra.")
    print()
    print("      Ví dụ:")
    print()
    print("        python tools/wheel_calib_calc.py spin --odom-deg 1794.3 --imu-deg 1801.1")
    print()
    print("=" * 74)
    print("  Mẹo: thêm `-h` sau mỗi chế độ để xem đầy đủ tham số, ví dụ:")
    print("        python tools/wheel_calib_calc.py roll -h")
    print("=" * 74)
    print()


def cmd_rev(a):
    N = a.counts / a.revolutions
    d_pc = math.pi * a.d / N
    print()
    print("=" * 62)
    print("  HIỆU CHUẨN N TỪ PHÉP QUAY TAY")
    print("=" * 62)
    print(f"  Tổng xung        : {a.counts}")
    print(f"  Số vòng          : {a.revolutions}")
    print(f"  ► N (xung/vòng)  = {N:.4f}")
    print(f"  ► d_pc           = {d_pc*1000:.6f} mm/xung")
    print(f"  ► xung/mét       = {1.0/d_pc:.2f}")
    print("-" * 62)

    nearest = round(N)
    if abs(N - nearest) < 0.02 * N:
        print(f"  ℹ️  N rất gần số nguyên {nearest}.")
        print(f"      Nếu sai khác chỉ do ±1 xung khi căn mốc, dùng {nearest} cũng được.")
        fact = factorise(nearest)
        if fact:
            print(f"      Phân tích {nearest} = {fact}  "
                  f"(có thể là PPR × tỉ số truyền × 4)")

    print()
    print("  → Trong config.h đặt:")
    print(f"      #define COUNTS_PER_WHEEL_REV   {N:.1f}f")
    print(f"      #define USE_MEASURED_DPC       0")
    print()
    print("  ⚠️  BẮT BUỘC làm tiếp phép thử `roll` để kiểm tra bán kính lăn hiệu dụng!")


def cmd_roll(a):
    d_pc = a.s_true / a.counts
    N_eff = math.pi * a.d / d_pc
    print()
    print("=" * 62)
    print("  HIỆU CHUẨN d_pc TỪ PHÉP CHẠY QUÃNG ĐƯỜNG ĐÃ BIẾT")
    print("=" * 62)
    print(f"  Quãng đường thực : {a.s_true:.4f} m")
    print(f"  Tổng xung        : {a.counts}")
    print(f"  ► d_pc           = {d_pc*1e6:.3f} µm/xung = {d_pc*1000:.6f} mm/xung")
    print(f"  ► xung/mét       = {1.0/d_pc:.2f}")
    print(f"  ► N hiệu dụng    = {N_eff:.3f} xung/vòng (ứng với D = {a.d*1000:.1f} mm)")
    print("-" * 62)

    if a.n_current:
        delta = (N_eff - a.n_current) / a.n_current * 100.0
        print(f"  So với N hiện tại ({a.n_current}): lệch {delta:+.2f}%")
        if abs(delta) < 1.0:
            print("  ✅ Sai khác < 1% → giữ USE_MEASURED_DPC = 0, không cần sửa gì.")
            print(f"      Lưu ý: d_pc = πD/N = {math.pi*a.d/a.n_current*1000:.6f} mm")
        else:
            print("  ⚠️  Sai khác ≥ 1% → nên dùng d_pc đo trực tiếp.")
    print()
    print("  → Trong config.h đặt:")
    print(f"      #define USE_MEASURED_DPC       1")
    print(f"      #define MEASURED_DPC_M         ({a.s_true:.4f}f / {a.counts:.1f}f)")
    print()
    print("  (Cách này ĐÃ bao gồm bán kính lăn hiệu dụng của lốp khi chịu tải")
    print("   → chính xác nhất, nên dùng cho báo cáo.)")


def cmd_spin(a):
    L_new = a.l_current * (a.odom_deg / a.imu_deg)
    err_before = (a.odom_deg - a.imu_deg) / a.imu_deg * 100.0
    print()
    print("=" * 62)
    print("  HIỆU CHUẨN L (KHOẢNG CÁCH 2 BÁNH) TỪ PHÉP QUAY TẠI CHỖ")
    print("=" * 62)
    print(f"  Góc theo ODOMETRY : {a.odom_deg:+.2f}°")
    print(f"  Góc theo IMU      : {a.imu_deg:+.2f}°")
    print(f"  Sai số hiện tại   : {err_before:+.2f}%")
    print("-" * 62)
    print(f"  L hiện tại        : {a.l_current*1000:.2f} mm")
    print(f"  ► L mới           = {L_new*1000:.2f} mm")
    print()
    print("  Công thức:  L_mới = L_cũ × (θ_odom / θ_imu)")
    print("  Lý do: ω_odom = Δs_diff / L, nên θ_odom ∝ 1/L.")
    print()
    print("  → Trong config.h đặt:")
    print(f"      #define WHEEL_TRACK_M          {L_new:.4f}f")
    print()
    print("  → Và cập nhật CÙNG GIÁ TRỊ trong URDF (base_link joint origin ±L/2)!")
    print()
    if abs(err_before) < 2.0:
        print("  ✅ Sai số < 2% → đã đạt yêu cầu nghiệm thu GĐ 3.")
    else:
        print("  ⚠️  Sai số ≥ 2% → nạp L mới rồi đo lại để xác nhận.")


def factorise(n):
    """Gợi ý phân tích n = PPR × ratio × 4."""
    if n % 4 != 0:
        return None
    core = n // 4
    out = []
    for ppr in range(2, 60):
        if core % ppr == 0:
            ratio = core // ppr
            if 5 <= ratio <= 500:
                out.append(f"{ppr}×{ratio}×4")
    return ", ".join(out[:6]) if out else None


def main():
    ap = argparse.ArgumentParser(
        description="Tính tham số hiệu chuẩn (docs/02 §7). "
                    "Chạy KHÔNG có tham số để xem hướng dẫn.")
    sub = ap.add_subparsers(dest="mode")

    p1 = sub.add_parser("rev", help="quay tay N vòng → N_enc")
    p1.add_argument("--counts", type=float, required=True, help="tổng xung đếm được")
    p1.add_argument("--revolutions", type=float, default=10.0)
    p1.add_argument("--d", type=float, default=0.065, help="đường kính bánh (m)")
    p1.set_defaults(func=cmd_rev)

    p2 = sub.add_parser("roll", help="chạy quãng đường đã biết → d_pc")
    p2.add_argument("--s-true", type=float, required=True, help="quãng đường thực (m)")
    p2.add_argument("--counts", type=float, required=True, help="tổng xung (trung bình 2 bánh)")
    p2.add_argument("--d", type=float, default=0.065)
    p2.add_argument("--n-current", type=float, default=1953,
                    help="N đang dùng trong firmware")
    p2.set_defaults(func=cmd_roll)

    p3 = sub.add_parser("spin", help="quay tại chỗ → L")
    p3.add_argument("--odom-deg", type=float, required=True)
    p3.add_argument("--imu-deg", type=float, required=True)
    p3.add_argument("--l-current", type=float, default=0.185, help="L hiện tại (m)")
    p3.set_defaults(func=cmd_spin)

    a = ap.parse_args()
    if a.mode is None:
        huong_dan()
        return
    a.func(a)
    print()


if __name__ == "__main__":
    main()
