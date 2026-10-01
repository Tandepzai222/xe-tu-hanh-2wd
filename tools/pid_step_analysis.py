#!/usr/bin/env python3
"""
pid_step_analysis.py — Nhận dạng mô hình động cơ (FOPDT) và tính gain PID theo IMC.

Công dụng
---------
Đọc file CSV xuất từ firmware (lệnh `mode step <pwm> <ms>` kèm `plot 100`),
tự động xác định:
    K_secant = độ lợi tĩnh đo được  [(m/s) trên 1 đơn vị PWM]
    tau      = hằng số thời gian [s]
    td       = thời gian trễ [s]
rồi tính gain PI theo phương pháp IMC-PI (docs/03 §6.1).

⚠️  K_secant ≠ K_độ_dốc
-----------------------
Phép đo step ở MỘT mức PWM chỉ cho K_secant = v_ss / u. Nhưng mô hình động cơ
có vùng chết dz thì:  v = K_độ_dốc · (u − dz).  Suy ra
    K_độ_dốc = v_ss / (u − dz)
Nếu nạp K_secant vào `cfg motor_k` thì feedforward sẽ THỪA PWM (xem giải thích
đầy đủ trong plot_telemetry.py). Vì vậy tool này có thêm `--dz` để in ra
K_độ_dốc — đó mới là số đúng để nạp vào config.h.

Cách dùng
---------
    # Trên Serial Monitor của firmware:
    #   > plot 100
    #   > mode step 400 3000
    # copy toàn bộ output CSV vào file step_400.csv

    python tools/pid_step_analysis.py step_400.csv
    python tools/pid_step_analysis.py step_400.csv --wheel r
    python tools/pid_step_analysis.py step_400.csv --lambda-factor 1.0 --plot
    python tools/pid_step_analysis.py step_400.csv --dz 0.14

Định dạng CSV hỗ trợ
--------------------
    t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R          (chuẩn của firmware này)
    t_ms,pwm_L,v_L,pwm_R,v_R                     (định dạng rút gọn)
"""

import argparse
import csv
import math
import os
import sys

# ⚠️ Phải chạy TRƯỚC mọi print/argparse: Windows mặc định dùng cp1252 nên
#    tiếng Việt sẽ gây UnicodeEncodeError. Xem tools/_console.py.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _console import setup_console          # noqa: E402
from _telemetry import doc_csv              # noqa: E402
setup_console()


PWM_MAX = 1023.0


def tao_csv_demo(path, K=0.37, tau=0.092, td=0.020,
                 duty=400, thoi_luong=3.0, ts=0.01):
    """
    Sinh file CSV step response TỔNG HỢP theo mô hình FOPDT đã biết:
        v(t) = 0  nếu t < td
        v(t) = K·u·(1 − e^(−(t−td)/τ))  nếu t ≥ td
    dùng để thử script khi chưa có dữ liệu thật.
    """
    import random
    random.seed(42)

    u = duty / PWM_MAX
    v_ss = K * u

    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R\n")
        t = 0.0
        while t <= thoi_luong:
            v = 0.0 if t < td else v_ss * (1.0 - math.exp(-(t - td) / tau))
            v += random.gauss(0.0, 0.002)          # nhiễu encoder nhẹ
            f.write(f"{int(round(t*1000))},STEP,{v_ss:.4f},{v:.4f},"
                    f"{v_ss:.4f},{v:.4f},{duty},{duty}\n")
            t += ts
    return path


def huong_dan():
    """In hướng dẫn khi chạy script không có tham số."""
    print()
    print("=" * 74)
    print("  pid_step_analysis.py — NHẬN DẠNG ĐỘNG CƠ + TÍNH GAIN PID (IMC)")
    print("=" * 74)
    print()
    print("  Script này đọc file CSV do firmware xuất ra, rồi tự tìm ra")
    print("    K_secant = độ lợi tĩnh (v_xác_lập / u)")
    print("    tau      = hằng số thời gian [s]")
    print("    td       = thời gian trễ [s]")
    print("  và tính gain PI theo phương pháp IMC-PI (docs/03 §6.1).")
    print()
    print("  ⚠️  Nếu bạn đã đo được vùng chết dz, hãy truyền --dz <giá trị>.")
    print("      Khi đó tool còn in ra K_độ_dốc = v_ss/(u − dz) — đó mới là số")
    print("      đúng để nạp vào `cfg motor_k` cho feedforward.")
    print()
    print("-" * 74)
    print("  QUY TRÌNH (làm trên Serial Monitor 115200 baud)")
    print("-" * 74)
    print()
    print("    1. Kê bánh lên giá cho quay tự do. Hiệu chuẩn trước:")
    print("         > cal imu")
    print("         > cal deadzone l")
    print()
    print("    2. Bật xuất CSV và chạy step:")
    print("         > plot 100")
    print("         > mode step 400 3000")
    print()
    print("    3. Copy TOÀN BỘ các dòng CSV in ra vào một file, ví dụ step_400.csv")
    print()
    print("    4. Chạy script này:")
    print()
    print("         python tools/pid_step_analysis.py step_400.csv")
    print("         python tools/pid_step_analysis.py step_400.csv --wheel r")
    print("         python tools/pid_step_analysis.py step_400.csv --dz 0.14")
    print("         python tools/pid_step_analysis.py step_400.csv --plot")
    print()
    print("-" * 74)
    print("  Không có firmware? Tạo file CSV mẫu để thử script:")
    print("    python tools/pid_step_analysis.py --demo")
    print("=" * 74)
    print()


def read_csv(path, wheel):
    """Đọc CSV, trả về (t[s], pwm[0..1023], v[m/s])."""
    rows = doc_csv(path, ["t_ms", "vL", "vR", "pwm_L", "pwm_R"])
    vcol = "vL" if wheel == "l" else "vR"
    pcol = "pwm_L" if wheel == "l" else "pwm_R"

    t0 = rows[0]["t_ms"]
    return (
        [(r["t_ms"] - t0) * 1e-3 for r in rows],
        [r[pcol] for r in rows],
        [r[vcol] for r in rows],
    )


def moving_average(x, n=5):
    """Lọc trung bình trượt. Trễ nhóm = (n−1)/2 · Ts."""
    if n <= 1:
        return list(x)
    out = []
    acc = 0.0
    for i, val in enumerate(x):
        acc += val
        if i >= n:
            acc -= x[i - n]
        out.append(acc / min(i + 1, n))
    return out


def thoi_diem_cat(t, v, nguong):
    """
    Thời điểm đầu tiên `v` vượt `nguong`, có NỘI SUY TUYẾN TÍNH giữa 2 mẫu.

    Vì sao bắt buộc nội suy: Ts = 10 ms mà τ ≈ 90 ms. Nếu chỉ lấy mẫu rời rạc
    thì sai số lượng tử hoá lên tới ±10 ms ≈ 11 % của τ — quá lớn để tune PID.
    """
    for i in range(1, len(v)):
        if v[i] >= nguong > v[i - 1]:
            dv = v[i] - v[i - 1]
            if abs(dv) < 1e-12:
                return t[i]
            return t[i - 1] + (nguong - v[i - 1]) / dv * (t[i] - t[i - 1])
    return None


def fit_fopdt(t, u, v):
    """
    Nhận dạng FOPDT bằng PHƯƠNG PHÁP HAI ĐIỂM 28.3 % / 63.2 %.

    Lý thuyết: đáp ứng bậc 1  v(t) = v_ss·(1 − e^(−(t−td)/τ))  đạt
        • 28.3 % của v_ss tại  t = td + τ/3
        • 63.2 % của v_ss tại  t = td + τ
    Suy ra:   τ  = 1.5 · (t_63.2 − t_28.3)
              td = t_63.2 − τ

    ⚠️ VÌ SAO KHÔNG DÙNG CÁCH "63.2 % trừ 2 %" NHƯ SÁCH GIÁO KHOA:
    Ta phải lọc nhiễu encoder bằng moving average n = 5. Lọc này là FIR pha
    tuyến tính nên dịch MỌI thời điểm cắt ngưỡng đi một lượng δ = (n−1)/2·Ts.
    Dùng HIỆU của hai mốc 28.3 % và 63.2 % thì δ TRIỆT TIÊU CHÍNH XÁC vì cả
    hai mốc cùng bị dịch một lượng như nhau. Riêng td vẫn bị dư δ nên ta trừ
    thẳng hằng số δ đã biết của bộ lọc.
    """
    # --- Trễ nhóm của bộ lọc trung bình trượt ---
    n_ma = 5
    if len(t) >= 2:
        ts = (t[-1] - t[0]) / float(len(t) - 1)
    else:
        ts = 0.01
    delay_ma = (n_ma - 1) / 2.0 * ts

    vf = moving_average(v, n_ma)

    # --- 1. Độ lợi tĩnh: trung bình 20 % mẫu cuối ---
    n_tail = max(5, len(vf) // 5)
    v_ss = sum(vf[-n_tail:]) / n_tail
    u_step = u[len(u) // 2]                      # giá trị PWM giữa giai đoạn
    u_norm = u_step / PWM_MAX

    if abs(u_norm) < 1e-6:
        sys.exit("LỖI: PWM bằng 0 — không thể nhận dạng.")
    if abs(v_ss) < 1e-4:
        sys.exit("LỖI: vận tốc xác lập gần bằng 0 — bánh không quay? "
                 "Kiểm tra test stand và nguồn.")

    K = v_ss / u_norm

    # --- 2. Hai mốc 28.3 % và 63.2 % ---
    t_283 = thoi_diem_cat(t, vf, 0.283 * v_ss)
    t_632 = thoi_diem_cat(t, vf, 0.632 * v_ss)

    if t_283 is None or t_632 is None:
        sys.exit("LỖI: tín hiệu không đạt 63.2 % giá trị xác lập — "
                 "thời gian ghi quá ngắn? Thử `mode step 400 4000`.")

    tau = 1.5 * (t_632 - t_283)
    if tau < 1e-3:
        sys.exit("LỖI: τ tính ra quá nhỏ — dữ liệu có vấn đề "
                 "(bánh quay trước khi có lệnh? nhiễu quá lớn?).")

    t_d = max(t_632 - tau - delay_ma, 0.0)

    # --- 3. Chất lượng mô hình: sai số RMS giữa đáp ứng ĐO và đáp ứng MÔ HÌNH ---
    sq = []
    for ti, vi in zip(t, vf):
        pred = 0.0 if ti < t_d else v_ss * (1.0 - math.exp(-(ti - t_d) / tau))
        sq.append((vi - pred) ** 2)
    rms = math.sqrt(sum(sq) / len(sq)) if sq else float("nan")
    fit_pct = (rms / abs(v_ss) * 100.0) if abs(v_ss) > 1e-9 else float("nan")

    return dict(K=K, tau=tau, t_d=t_d, v_ss=v_ss, u_norm=u_norm,
                fit_pct=fit_pct, v=v, t=t,
                t_283=t_283, t_632=t_632, delay_ma=delay_ma)


def imc_pi(K, tau, t_d, lam):
    """IMC-PI: Kp = tau / (K*(lam+td)),  Ti = tau."""
    Kp = tau / (K * (lam + t_d))
    Ti = tau
    Ki = Kp / Ti
    return Kp, Ki, Ti


def rise_time(t, v):
    """Thời gian lên 10% → 90%."""
    v_ss = sum(v[-max(5, len(v)//5):]) / max(5, len(v)//5)
    t10 = t90 = None
    for ti, vi in zip(t, v):
        if t10 is None and vi >= 0.10 * v_ss:
            t10 = ti
        if t90 is None and vi >= 0.90 * v_ss:
            t90 = ti
            break
    if t10 is None or t90 is None:
        return float("nan")
    return t90 - t10


def main():
    ap = argparse.ArgumentParser(
        description="Nhận dạng động cơ FOPDT + tính gain PID theo IMC (docs/03 §5-6). "
                    "Chạy KHÔNG có tham số để xem hướng dẫn.")
    ap.add_argument("csv", nargs="?", help="file CSV xuất từ firmware")
    ap.add_argument("--wheel", choices=["l", "r"], default="l",
                    help="bánh nào (mặc định l)")
    ap.add_argument("--lambda-factor", type=float, default=1.0,
                    help="λ = factor × τ (mặc định 1.0). Nhỏ hơn = nhanh hơn nhưng overshoot.")
    ap.add_argument("--plot", action="store_true", help="vẽ đồ thị (cần matplotlib)")
    ap.add_argument("--demo", action="store_true",
                    help="tự sinh dữ liệu mẫu rồi phân tích (thử script không cần robot)")
    # SỬA (lỗi 5): thêm --dz để tính K_độ_dốc đúng cho feedforward
    ap.add_argument("--dz", type=float, default=0.0,
                    help="vùng chết PWM (0..1). Nếu > 0, in thêm K_độ_dốc "
                         "= v_ss/(u−dz) — số đúng để nạp vào `cfg motor_k`.")
    args = ap.parse_args()

    # ---- Chế độ tự kiểm tra: sinh dữ liệu rồi phân tích ngay ----
    if args.demo:
        import tempfile
        path = os.path.join(tempfile.gettempdir(), "step_demo.csv")
        tao_csv_demo(path)
        print(f"\n[DEMO] Đã sinh dữ liệu mẫu: {path}")
        print("[DEMO] Mô hình gốc đặt vào:  K = 0.37,  tau = 0.092 s,  td = 0.020 s,")
        print("[DEMO]                       PWM = 400/1023")
        print("[DEMO] Nếu script nhận dạng ra đúng các số trên → công cụ hoạt động tốt.")
        args.csv = path

    if args.csv is None:
        huong_dan()
        return

    t, u, v = read_csv(args.csv, args.wheel)
    res = fit_fopdt(t, u, v)

    K, tau, t_d = res["K"], res["tau"], res["t_d"]

    print()
    print("=" * 68)
    print(f"  NHẬN DẠNG ĐỘNG CƠ — bánh {'TRÁI' if args.wheel == 'l' else 'PHẢI'}")
    print("=" * 68)
    print(f"  File            : {args.csv}")
    print(f"  Số mẫu          : {len(t)}")
    print(f"  Thời gian ghi   : {t[-1]:.3f} s")
    print(f"  PWM bước        : {res['u_norm'] * PWM_MAX:.0f} / {PWM_MAX:.0f}  "
          f"(u = {res['u_norm']:.4f})")
    print(f"  Vận tốc xác lập : {res['v_ss']:.4f} m/s")
    print("-" * 68)
    print(f"  ► K_secant      = {K:.4f}   (m/s) / PWM   ← đo được từ step")
    print(f"  ► tau           = {tau * 1000:.1f} ms")
    print(f"  ► td            = {t_d * 1000:.1f} ms")
    print(f"  ► t_rise(10-90) = {rise_time(t, res['v']) * 1000:.1f} ms")

    # SỬA (lỗi 5): nếu biết dz, in thêm K_độ_dốc — số đúng cho feedforward
    if args.dz > 0.0:
        u_eff = res["u_norm"] - args.dz
        if u_eff <= 1e-6:
            print(f"  ⚠️  dz = {args.dz:.3f} ≥ u = {res['u_norm']:.4f} → "
                  f"không tính được K_độ_dốc (vùng chết lớn hơn PWM bước).")
        else:
            K_slope = res["v_ss"] / u_eff
            print(f"  ► K_độ_dốc      = {K_slope:.4f}   "
                  f"= v_ss / (u − dz)   ← NẠP VÀO `cfg motor_k`")
            print(f"    (dùng K_độ_dốc này cho feedforward, KHÔNG dùng K_secant)")

    ratio = tau / t_d if t_d > 1e-6 else float("inf")
    print(f"  ► tau/td        = {ratio:.2f}")
    print("-" * 68)

    if ratio < 3.0:
        verdict = "RẤT KHÓ điều khiển → chỉ PI, gain khiêm tốn"
    elif ratio < 10.0:
        verdict = "TRUNG BÌNH → dùng PI, KHÔNG dùng D   ← trường hợp điển hình"
    else:
        verdict = "DỄ điều khiển → có thể thêm khâu D"
    print(f"  Đánh giá: {verdict}")

    fit = res["fit_pct"]
    if not math.isnan(fit):
        if fit < 2.0:
            print(f"  Chất lượng mô hình bậc 1: sai số RMS = {fit:.2f} % v_ss   ✅ rất tốt")
        elif fit < 5.0:
            print(f"  Chất lượng mô hình bậc 1: sai số RMS = {fit:.2f} % v_ss   ✅ chấp nhận được")
        else:
            print(f"  Chất lượng mô hình bậc 1: sai số RMS = {fit:.2f} % v_ss   ⚠️ kém")
            print("      → Mô hình bậc 1 KHÔNG mô tả hết hành vi động cơ. Nguyên nhân thường gặp:")
            print("        • Vùng chết PWM chưa bù hết (chạy `cal deadzone <l|r>`)")
            print("        • Ma sát Coulomb / caster trượt")
            print("        • Bão hoà PWM (PWM quá cao — thử `mode step 300 3000`)")
            print("        • Bánh chưa quay tự do (còn chạm đất)")
            print("      Gain IMC bên dưới vẫn dùng được làm ĐIỂM KHỞI ĐẦU, nhưng phải")
            print("      tinh chỉnh thủ công theo quy trình docs/03 §9.")

    print()
    print("=" * 68)
    print("  GAIN KHỞI ĐIỂM THEO IMC-PI   (Kp = τ/(K(λ+td)) ,  Ki = Kp/τ)")
    print("=" * 68)
    print(f"  {'λ':>10}  {'λ/τ':>6}  {'Kp':>8}  {'Ki':>9}  {'Ki·Ts':>8}   ghi chú")
    print("  " + "-" * 64)

    for factor, note in [(0.2, "RẤT nhanh, overshoot lớn"),
                         (0.5, "nhanh, overshoot ~10%"),
                         (1.0, "cân bằng  ← KHUYẾN NGHỊ"),
                         (2.0, "mượt, rất bền vững"),
                         (4.0, "bám chậm")]:
        lam = factor * tau
        Kp, Ki, _ = imc_pi(K, tau, t_d, lam)
        mark = " ◄" if abs(factor - args.lambda_factor) < 1e-9 else ""
        print(f"  {lam:10.4f}  {factor:6.1f}  {Kp:8.3f}  {Ki:9.3f}  "
              f"{Ki*0.01:8.4f}   {note}{mark}")

    lam = args.lambda_factor * tau
    Kp, Ki, Ti = imc_pi(K, tau, t_d, lam)
    loop_gain = Kp * K

    print()
    print("=" * 68)
    print(f"  KHUYẾN NGHỊ NẠP VÀO FIRMWARE  (λ = {args.lambda_factor:.1f}·τ = {lam:.4f} s)")
    print("=" * 68)
    w = args.wheel
    print(f"    pid {w} kp {Kp:.3f}")
    print(f"    pid {w} ki {Ki:.3f}")
    print(f"    pid {w} kd 0.000")
    # SỬA (lỗi 5): nhắc luôn giá trị motor_k đúng khi biết dz
    if args.dz > 0.0 and (res["u_norm"] - args.dz) > 1e-6:
        K_slope = res["v_ss"] / (res["u_norm"] - args.dz)
        print(f"    cfg motor_k {K_slope:.4f}   ← dùng K_độ_dốc, KHÔNG dùng {K:.4f}")
    print()
    print(f"  Loop gain Kp·K = {loop_gain:.3f}   (gần 1 là tốt; > 3 dễ dao động)")
    print(f"  Ti = {Ti*1000:.1f} ms")
    print()
    print("  Sau khi nạp, kiểm tra bằng `mode wheel 0.2 0.2` và `plot 100`.")
    print("  Mục tiêu: sai số xác lập < 3%, overshoot < 5%, t_rise < 250 ms.")
    print()
    print("  ⚠️ Đừng quên feedforward! Chạy trước với Kp=Ki=0 để chỉnh")
    print("     `cfg dzl`/`cfg dzr` và `cfg motor_k`, rồi mới bật PID.")
    print("=" * 68)

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
            ax[0].plot(t, v, lw=1.5, label="v đo được")
            ax[0].axhline(res["v_ss"], color="g", ls="--", lw=1, label="v_xác lập")
            ax[0].axhline(0.632 * res["v_ss"], color="orange", ls=":", lw=1,
                          label="63.2% (→ τ)")
            ax[0].axvline(t_d, color="r", ls=":", lw=1, label=f"td = {t_d*1000:.0f} ms")
            ax[0].set_ylabel("v (m/s)")
            ax[0].legend()
            ax[0].grid(alpha=0.3)
            ax[0].set_title(f"Nhận dạng FOPDT — K={K:.3f}, τ={tau*1000:.0f} ms, "
                            f"td={t_d*1000:.0f} ms")

            ax[1].plot(t, [x / PWM_MAX for x in u], color="purple", lw=1.5)
            ax[1].set_ylabel("PWM (chuẩn hoá)")
            ax[1].set_xlabel("thời gian (s)")
            ax[1].grid(alpha=0.3)
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("\n[!] Không có matplotlib → bỏ qua vẽ đồ thị. "
                  "Cài bằng: pip install matplotlib\n")


if __name__ == "__main__":
    main()