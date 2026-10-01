#!/usr/bin/env python3
"""
plot_telemetry.py — Phân tích đáp ứng vòng kín của PID và vẽ đồ thị.

Công dụng
---------
Đọc CSV xuất từ firmware (`plot 100` → copy vào file), rồi:
    • Tính các chỉ tiêu chất lượng: overshoot, thời gian lên, thời gian xác lập,
      sai số xác lập, sai lệch 2 bánh, IAE, ISE.
    • So sánh với tiêu chí nghiệm thu trong docs/03 §9.8.
    • Vẽ đồ thị: vận tốc, PWM, sai số.

Ngoài ra còn có 2 chế độ KHÔNG cần robot:
    • --demo   : mô phỏng vòng kín rồi phân tích (tự kiểm tra công cụ).
    • --sweep  : quét λ ∈ [0.25τ, 8τ], chọn bộ gain IMC-PI ĐẠT tiêu chí
                 nghiệm thu với λ nhỏ nhất (đáp ứng nhanh nhất mà vẫn đạt).

Cách dùng
---------
    python tools/plot_telemetry.py telemetry.csv
    python tools/plot_telemetry.py telemetry.csv --target 0.2
    python tools/plot_telemetry.py telemetry.csv --no-plot
    python tools/plot_telemetry.py --demo
    python tools/plot_telemetry.py --sweep --K-slope 0.576 --tau 0.092 --td 0.020 --dz 0.14

Định dạng CSV
-------------
    t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R
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
from _telemetry import doc_dong_csv, chuan_hoa_ten_cot   # noqa: E402
setup_console()


PWM_MAX = 1023.0


def huong_dan():
    """In hướng dẫn khi chạy script không có tham số."""
    print()
    print("=" * 74)
    print("  plot_telemetry.py — ĐÁNH GIÁ CHẤT LƯỢNG PID TỪ CSV TELEMETRY")
    print("=" * 74)
    print()
    print("  Script này đọc file CSV do firmware xuất, rồi tính:")
    print("    • Overshoot            • Thời gian lên (10-90 %)")
    print("    • Thời gian xác lập    • Sai số xác lập")
    print("    • IAE, ISE             • Sai lệch giữa 2 bánh")
    print("  và ĐỐI CHIẾU với tiêu chí nghiệm thu ở docs/03 §9.8.")
    print()
    print("-" * 74)
    print("  QUY TRÌNH")
    print("-" * 74)
    print()
    print("    1. Nạp gain vào firmware:")
    print("         > pid l kp 2.2")
    print("         > pid l ki 24")
    print("         > pid r kp 2.2")
    print("         > pid r ki 24")
    print()
    print("    2. Kê bánh lên giá, rồi bật CSV và chạy:")
    print("         > plot 100")
    print("         > mode wheel 0.2 0.2")
    print()
    print("    3. Đợi ~3 giây cho vận tốc ổn định, rồi:")
    print("         > stop")
    print("         > plot off")
    print()
    print("    4. Copy các dòng CSV vào file telemetry.csv rồi chạy:")
    print()
    print("         python tools/plot_telemetry.py telemetry.csv")
    print("         python tools/plot_telemetry.py telemetry.csv --target 0.2 --plot")
    print()
    print("-" * 74)
    print("  KHÔNG CÓ DỮ LIỆU THẬT?")
    print("    python tools/plot_telemetry.py --demo     # mô phỏng vòng kín")
    print()
    print("  MUỐN QUÉT λ ĐỂ TÌM GAIN ĐẠT TIÊU CHÍ?")
    print("    python tools/plot_telemetry.py --sweep \\")
    print("        --K-slope 0.576 --tau 0.092 --td 0.020 --dz 0.14 --target 0.2")
    print("=" * 74)
    print()


def mo_phong(K_slope=0.576, tau=0.092, td=0.020, dz=0.14,
             kp=2.20, ki=24.0, kd=0.0,
             target=0.20, thoi_luong=3.0, ts=0.01, acc_max=1.0):
    """
    MÔ PHỎNG vòng kín: slew limiter + feedforward + PID rời rạc + mô hình FOPDT.
    Trả về danh sách các bộ (t, setpoint, vận tốc, pwm_chuẩn_hoá).

    Chuỗi xử lý bám sát robot.cpp step() §8–9:
        1. slew limiter   giới hạn gia tốc ACC_MAX_MPS2
        2. feedforward    u_ff = dz·sign(vsp) + vsp/K_slope
        3. PID            đạo hàm theo đo lường, có lọc, anti-windup kiểu
                          "conditional integration"
        4. bão hoà        |u| ≤ 1
        5. động cơ        τ·dv/dt = K_slope·(u − dz·sign(u)) − v

    ⚠️⚠️  HAI CÁI BẪY — ĐỌC KỸ TRƯỚC KHI TỰ VIẾT MÔ PHỎNG  ⚠️⚠️

    BẪY 1 — QUÊN SLEW LIMITER.
      Nếu cho setpoint nhảy bậc, feedforward nhảy theo tức thì trong khi động cơ
      còn trễ → ngõ ra bão hoà |u| = 1 trong lúc chờ → mô phỏng báo overshoot
      ~20 % một cách SAI LỆCH. Robot thật không như vậy vì tần số đặt luôn bị
      giới hạn gia tốc trước khi tới PID.

    BẪY 2 — NHẦM `K` SECANT VỚI `K` ĐỘ DỐC.  ← bẫy nguy hiểm hơn nhiều
      Phép đo step ở MỘT mức PWM chỉ cho ta độ lợi SECANT:

            K_secant = v_xác_lập / u        (ví dụ 0.1446 / 0.391 = 0.37)

      Nhưng mô hình động cơ có vùng chết là  v = K_độ_dốc·(u − dz).  Suy ra

            K_độ_dốc = v_xác_lập / (u − dz)   (ví dụ 0.1446 / 0.251 = 0.576)

      Nếu bạn nạp `MOTOR_K = 0.37` (secant) vào feedforward  u_ff = dz + vsp/K
      thì feedforward sẽ THỪA PWM:

            u_ff = 0.14 + 0.20/0.37 = 0.681   →   v = 0.576·(0.681−0.14) = 0.312 m/s
                                                                              ↑
                                              trong khi setpoint chỉ là 0.20 m/s
                                                        → OVERSHOOT 56 % !!

      Cách chữa: đo `dz` bằng `cal deadzone`, rồi tính
      `K_độ_dốc = v_xác_lập/(u − dz)` và nạp GIÁ TRỊ ĐÓ vào `cfg motor_k`.
      Công cụ `pid_step_analysis.py --dz 0.14` in ra K_độ_dốc tự động.
    """
    import random
    random.seed(7)

    n_delay = max(1, int(round(td / ts)))
    delay_buf = [0.0] * n_delay

    v = 0.0
    sp = 0.0                                  # setpoint SAU slew limiter
    integ = 0.0
    i_min, i_max = -0.60, 0.60
    d_filt = 0.0
    prev_meas = 0.0
    d_tau = 0.020

    rows = []
    t = 0.0
    while t <= thoi_luong:
        sp_target = 0.0 if t < 0.2 else target      # bước nhảy tại t = 0.2 s

        # ---- 1. slew limiter ----
        dv = acc_max * ts
        if sp_target > sp + dv:
            sp += dv
        elif sp_target < sp - dv:
            sp -= dv
        else:
            sp = sp_target

        # ---- 2. feedforward ----
        ff = 0.0
        if abs(sp) > 0.005:
            ff = math.copysign(dz + abs(sp) / K_slope, sp)

        # ---- 3. PID ----
        e = sp - v
        p = kp * e
        integ += ki * ts * e
        integ = max(i_min, min(i_max, integ))

        d_meas = (v - prev_meas) / ts if ts > 1e-9 else 0.0
        alpha = d_tau / (d_tau + ts)
        d_filt = alpha * d_filt + (1 - alpha) * d_meas
        d = -kd * d_filt
        prev_meas = v

        u = p + integ + d + ff

        # ---- 4. bão hoà + anti-windup conditional integration ----
        u_sat = max(-1.0, min(1.0, u))
        if u != u_sat:
            integ -= ki * ts * e
            integ = max(i_min, min(i_max, integ))

        # ---- 5. động cơ có vùng chết + trễ ----
        u_delayed = delay_buf.pop(0)
        delay_buf.append(u_sat)

        if abs(u_delayed) <= dz:
            u_eff = 0.0                       # chưa thắng được vùng chết
        else:
            u_eff = u_delayed - math.copysign(dz, u_delayed)

        v += ts * (K_slope * u_eff - v) / tau
        v += random.gauss(0.0, 0.0015)        # nhiễu encoder

        rows.append((t, sp, v, u_sat))
        t += ts

    return rows


def ghi_csv(path, rows, mode="WHEEL"):
    """Ghi kết quả mô phỏng ra file CSV CÙNG ĐỊNH DẠNG với firmware."""
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R\n")
        for (ti, spi, vi, ui) in rows:
            pwm = int(round(ui * PWM_MAX))
            f.write(f"{int(round(ti*1000))},{mode},{spi:.4f},{vi:.4f},"
                    f"{spi:.4f},{vi:.4f},{pwm},{pwm}\n")
    return path


# SỬA (lỗi 1): hàm này trước đây được gọi trong --demo nhưng KHÔNG tồn tại.
# Giờ định nghĩa nó ngay tại đây, dùng `mo_phong` + `ghi_csv` có sẵn.
def tao_csv_demo(path, target=0.20, thoi_luong=3.0):
    """
    Sinh CSV telemetry TỔNG HỢP bằng cách mô phỏng vòng kín, để `--demo`
    có dữ liệu phân tích mà không cần robot.
    """
    rows = mo_phong(kp=2.20, ki=24.0, kd=0.0,
                    target=target, thoi_luong=thoi_luong)
    return ghi_csv(path, rows, mode="WHEEL")


def imc_pi_gain(K_slope, tau, td, lam):
    """IMC-PI: Kp = τ/(K·(λ+td)),  Ki = Kp/τ.  Dùng K ĐỘ DỐC, không phải K secant."""
    kp = tau / (K_slope * (lam + td))
    return kp, kp / tau


def che_do_sweep(args):
    """
    QUÉT λ ĐỂ TÌM BỘ GAIN ĐẠT TIÊU CHÍ NGHIỆM THU.

    Mô phỏng vòng kín với nhiều giá trị λ rồi chọn bộ đạt yêu cầu ở
    docs/03 §9.8:  overshoot < 5 %,  t_lên < 250 ms,  t_xác_lập < 500 ms.
    """
    print()
    print("=" * 82)
    print("  QUÉT λ — TÌM BỘ GAIN IMC-PI ĐẠT TIÊU CHÍ NGHIỆM THU")
    print("=" * 82)
    print(f"  Mô hình động cơ : K_độ_dốc = {args.K_slope:.4f}   τ = {args.tau*1000:.0f} ms"
          f"   td = {args.td*1000:.0f} ms   dz = {args.dz:.3f}")
    print(f"  Setpoint        : {args.target:.3f} m/s   (bước nhảy, có slew limiter"
          f" {args.acc_max:.2f} m/s²)")
    print(f"  Tiêu chí        : overshoot < 5 %   |   t_lên < 250 ms   |   t_xác_lập < 500 ms")
    print("-" * 82)
    print(f"  {'λ/τ':>5} {'Kp':>8} {'Ki':>8} {'Ki·Ts':>7} "
          f"{'overshoot':>10} {'t_lên':>9} {'t_xác_lập':>10} {'IAE':>9}  kết quả")
    print("  " + "-" * 78)

    tot = None
    for factor in [0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0]:
        lam = factor * args.tau
        kp, ki = imc_pi_gain(args.K_slope, args.tau, args.td, lam)

        rows = mo_phong(K_slope=args.K_slope, tau=args.tau, td=args.td, dz=args.dz,
                        kp=kp, ki=ki, kd=0.0, target=args.target,
                        thoi_luong=args.thoi_luong, acc_max=args.acc_max)
        t = [r[0] for r in rows]
        v = [r[2] for r in rows]
        r = analyse(t, [args.target] * len(t), v, "sim")

        ok_ov = r["overshoot"] < 5.0
        ok_rt = r["t_rise"] * 1000 < 250.0
        ok_st = r["t_settle"] * 1000 < 500.0
        ok = ok_ov and ok_rt and ok_st

        mark = "✅ ĐẠT" if ok else ("❌ overshoot" if not ok_ov else "❌ chậm")
        print(f"  {factor:5.2f} {kp:8.3f} {ki:8.3f} {ki*0.01:7.4f} "
              f"{r['overshoot']:9.2f}% {r['t_rise']*1000:8.0f}ms "
              f"{r['t_settle']*1000:9.0f}ms {r['iae']:9.4f}  {mark}")

        if ok and tot is None:
            tot = (factor, kp, ki, r)

    print("-" * 82)
    if tot is None:
        print("  ⚠️  KHÔNG có λ nào trong dải quét đạt cả 3 tiêu chí.")
        print("      Nguyên nhân thường gặp:")
        print("        • Mô hình động cơ chưa đúng (đo lại K_slope, τ, td)")
        print("        • dz chưa bù hết → nạp `cfg dz` đúng trước")
        print("        • Nhiễu encoder quá lớn so với tín hiệu")
        print("      Trong trường hợp này hãy nới tiêu chí hoặc tinh chỉnh tay theo docs/03 §9.")
    else:
        factor, kp, ki, r = tot
        print(f"  ✅ KHUYẾN NGHỊ: λ = {factor:.2f}·τ")
        print()
        print(f"       pid l kp {kp:.3f}")
        print(f"       pid l ki {ki:.3f}")
        print(f"       pid l kd 0.000")
        print(f"       (tương tự cho bánh r)")
        print()
        print(f"     Kết quả mô phỏng: overshoot {r['overshoot']:.2f} %, "
              f"t_lên {r['t_rise']*1000:.0f} ms, t_xác_lập {r['t_settle']*1000:.0f} ms, "
              f"IAE {r['iae']:.4f}")
        print(f"     Loop gain Kp·K_độ_dốc = {kp*args.K_slope:.3f}")
    print()
    print("  ⚠️  Mô phỏng KHÔNG thay thế thực nghiệm: nó bỏ qua ma sát tĩnh,")
    print("      sự khác nhau giữa 2 bánh, và nhiễu điện. Hãy dùng kết quả này làm")
    print("      ĐIỂM KHỞI ĐẦU, rồi tinh chỉnh trên robot thật theo docs/03 §9.")
    print("=" * 82)
    print()


def read_csv(path):
    lines = doc_dong_csv(path)
    if not lines:
        raise SystemExit(
            f"LỖI: không tìm thấy dòng dữ liệu nào trong '{path}'.\n"
            f"     Hãy bật `plot 100` trên firmware rồi copy output vào file."
        )

    rdr = csv.DictReader(lines)
    # SỬA (lỗi 3): chuẩn hoá fieldnames TRƯỚC khi đọc để tránh KeyError
    # khi header có space (ví dụ "t_ms, md, vL").
    rdr.fieldnames = [chuan_hoa_ten_cot(c) for c in (rdr.fieldnames or [])]
    cols = list(rdr.fieldnames)

    need = ["t_ms", "vspL", "vL", "vspR", "vR", "pwm_L", "pwm_R"]
    thieu = [c for c in need if c not in cols]
    if thieu:
        raise SystemExit(
            f"LỖI: file '{path}' thiếu cột {thieu}.\n"
            f"     Cột tìm thấy: {cols}\n"
            f"     Cần định dạng: t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R"
        )

    out = {k: [] for k in need}
    out["md"] = []
    for r in rdr:
        try:
            # SỬA (lỗi 3): bắt thêm KeyError để một dòng lạ không làm chết tool
            vals = [float(r[k]) for k in need]
        except (TypeError, ValueError, KeyError):
            continue
        for k, val in zip(need, vals):
            out[k].append(val)
        out["md"].append((r.get("md") or "").strip())

    if len(out["t_ms"]) < 20:
        raise SystemExit(f"LỖI: chỉ có {len(out['t_ms'])} mẫu hợp lệ.")

    t0 = out["t_ms"][0]
    out["t"] = [(x - t0) * 1e-3 for x in out["t_ms"]]
    return out


def analyse(t, sp, y, label, tol_band=0.05):
    """Tính các chỉ tiêu chất lượng của một bước đáp ứng."""
    if not y:
        return None

    target = sp[-1]
    if abs(target) < 1e-6:
        return None

    # --- Overshoot ---
    peak = max(y) if target > 0 else min(y)
    overshoot = (peak - target) / target * 100.0
    if overshoot < 0:
        overshoot = 0.0

    # --- Thời gian lên 10% → 90% ---
    t10 = t90 = None
    for ti, yi in zip(t, y):
        if t10 is None and yi >= 0.10 * target:
            t10 = ti
        if t90 is None and yi >= 0.90 * target:
            t90 = ti
            break
    t_rise = (t90 - t10) if (t10 is not None and t90 is not None) else float("nan")

    # --- Thời gian xác lập (±5%) ---
    lo, hi = target * (1 - tol_band), target * (1 + tol_band)
    t_settle = float("nan")
    for i in range(len(y)):
        if all(lo <= v <= hi for v in y[i:]):
            t_settle = t[i]
            break

    # --- Sai số xác lập: trung bình 20% mẫu cuối ---
    n_tail = max(5, len(y) // 5)
    ss_err = (sum(y[-n_tail:]) / n_tail - target) / target * 100.0

    # --- IAE / ISE ---
    iae = ise = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.5:
            continue
        e = target - y[i]
        iae += abs(e) * dt
        ise += e * e * dt

    return dict(label=label, target=target, overshoot=overshoot, t_rise=t_rise,
                t_settle=t_settle, ss_err=ss_err, iae=iae, ise=ise)


def check(name, value, limit, unit, lower_is_better=True, ok_msg="ĐẠT"):
    if value is None or value != value:      # NaN
        print(f"  {name:<28} {'---':>12} {unit:<6}  (không đo được)")
        return
    ok = (value < limit) if lower_is_better else (value > limit)
    mark = "✅" if ok else "❌"
    print(f"  {name:<28} {value:>12.3f} {unit:<6}  "
          f"(ngưỡng {'<' if lower_is_better else '>'} {limit})  {mark}")


def main():
    ap = argparse.ArgumentParser(
        description="Phân tích đáp ứng PID từ CSV telemetry. "
                    "Chạy KHÔNG có tham số để xem hướng dẫn.")
    ap.add_argument("csv", nargs="?", help="file CSV xuất từ firmware")
    ap.add_argument("--target", type=float, default=None,
                    help="setpoint (m/s). Mặc định lấy giá trị cuối của vspL")
    ap.add_argument("--no-plot", action="store_true", help="không vẽ đồ thị")
    ap.add_argument("--demo", action="store_true",
                    help="mô phỏng vòng kín rồi phân tích (thử script không cần robot)")

    # SỬA (lỗi 2): các option dưới đây trước đây THIẾU, khiến `che_do_sweep`
    # không bao giờ chạy được (dù hàm đã được viết).
    ap.add_argument("--sweep", action="store_true",
                    help="quét λ để tìm gain IMC-PI đạt tiêu chí nghiệm thu "
                         "(không cần file CSV)")
    ap.add_argument("--K-slope", type=float, default=0.576,
                    help="[sweep] K_độ_dốc (KHÔNG phải K_secant). Mặc định 0.576")
    ap.add_argument("--tau", type=float, default=0.092,
                    help="[sweep] hằng số thời gian (s). Mặc định 0.092")
    ap.add_argument("--td", type=float, default=0.020,
                    help="[sweep] thời gian trễ (s). Mặc định 0.020")
    ap.add_argument("--dz", type=float, default=0.14,
                    help="[sweep] vùng chết PWM (0..1). Mặc định 0.14")
    ap.add_argument("--thoi-luong", type=float, default=3.0,
                    help="[sweep] thời lượng mô phỏng (s). Mặc định 3.0")
    ap.add_argument("--acc-max", type=float, default=1.0,
                    help="[sweep] giới hạn gia tốc của slew limiter (m/s²). Mặc định 1.0")
    args = ap.parse_args()

    # ---- Chế độ quét λ (ưu tiên cao nhất — không cần CSV) ----
    if args.sweep:
        if args.target is None:
            args.target = 0.20
        che_do_sweep(args)
        return

    # ---- Chế độ tự kiểm tra: mô phỏng vòng kín ----
    if args.demo:
        import tempfile
        path = os.path.join(tempfile.gettempdir(), "telemetry_demo.csv")
        # SỬA (lỗi 1): tao_csv_demo giờ đã được định nghĩa trong file này
        tao_csv_demo(path)
        print(f"\n[DEMO] Đã mô phỏng vòng kín: {path}")
        print("[DEMO] Mô hình động cơ: K_độ_dốc=0.576, tau=92 ms, td=20 ms, dz=0.14")
        print("[DEMO] Gain PID:        Kp=2.20, Ki=24.0, Kd=0  (giá trị mặc định config.h)")
        print("[DEMO] Setpoint:        0.20 m/s (bước nhảy tại t = 0.2 s)")
        args.csv = path

    if args.csv is None:
        huong_dan()
        return

    d = read_csv(args.csv)
    t = d["t"]

    target = args.target if args.target is not None else d["vspL"][-1]
    # tạo setpoint tham chiếu hằng số
    sp = [target] * len(t)

    rl = analyse(t, sp, d["vL"], "Trái")
    rr = analyse(t, sp, d["vR"], "Phải")

    print()
    print("=" * 72)
    print(f"  PHÂN TÍCH ĐÁP ỨNG PID   —   setpoint = {target:.4f} m/s")
    print("=" * 72)
    print(f"  File          : {args.csv}")
    print(f"  Số mẫu        : {len(t)}")
    print(f"  Thời gian     : {t[-1]:.3f} s")
    print(f"  Chế độ        : {d['md'][len(d['md'])//2] if d['md'] else '?'}")
    print("-" * 72)

    for r, wheel in [(rl, "TRÁI"), (rr, "PHẢI")]:
        if r is None:
            continue
        print(f"  --- Bánh {wheel} ---")
        print(f"    Overshoot              : {r['overshoot']:8.2f} %")
        print(f"    Thời gian lên (10-90%) : {r['t_rise']*1000:8.1f} ms")
        print(f"    Thời gian xác lập ±5%  : {r['t_settle']*1000:8.1f} ms")
        print(f"    Sai số xác lập         : {r['ss_err']:+8.2f} %")
        print(f"    IAE                    : {r['iae']:8.4f}  (m/s)·s")
        print(f"    ISE                    : {r['ise']:8.4f}  (m/s)²·s")
        print()

    print("-" * 72)
    print("  ĐỐI CHIẾU TIÊU CHÍ NGHIỆM THU (docs/03 §9.8)")
    print("-" * 72)
    for r, wheel in [(rl, "TRÁI"), (rr, "PHẢI")]:
        if r is None:
            continue
        print(f"  [Bánh {wheel}]")
        check("Sai số xác lập |%|", abs(r["ss_err"]), 3.0, "%")
        check("Overshoot", r["overshoot"], 5.0, "%")
        check("Thời gian lên", r["t_rise"] * 1000, 250.0, "ms")
        check("Thời gian xác lập", r["t_settle"] * 1000, 500.0, "ms")
        print()

    if rl and rr:
        # Sai lệch giữa 2 bánh ở giai đoạn xác lập
        n_tail = max(5, len(t) // 5)
        mL = sum(d["vL"][-n_tail:]) / n_tail
        mR = sum(d["vR"][-n_tail:]) / n_tail
        diff = abs(mL - mR) / abs(target) * 100.0 if abs(target) > 1e-6 else 0.0
        print(f"  [Đối xứng 2 bánh]")
        check("Sai lệch vL vs vR", diff, 2.0, "%")
        print(f"    vL_tb = {mL:.4f} m/s   vR_tb = {mR:.4f} m/s   "
              f"(chênh {abs(mL-mR)*1000:.2f} mm/s)")
        print()

    # --- Thống kê PWM ---
    satL = sum(1 for x in d["pwm_L"] if abs(x) >= PWM_MAX * 0.99)
    satR = sum(1 for x in d["pwm_R"] if abs(x) >= PWM_MAX * 0.99)
    print("-" * 72)
    print(f"  PWM bão hoà (|u| ≥ 99%): L = {satL}/{len(t)} mẫu, R = {satR}/{len(t)} mẫu")
    if satL > len(t) * 0.10 or satR > len(t) * 0.10:
        print("  ⚠️  Bão hoà PWM đáng kể → setpoint quá cao hoặc gain quá lớn.")
    print("=" * 72)
    print()

    if args.no_plot:
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] Không có matplotlib → bỏ qua vẽ đồ thị. pip install matplotlib\n")
        return

    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    ax[0].plot(t, d["vspL"], "k--", lw=1, label="setpoint")
    ax[0].plot(t, d["vL"], lw=1.6, label="v bánh TRÁI")
    ax[0].plot(t, d["vR"], lw=1.6, label="v bánh PHẢI")
    ax[0].set_ylabel("v (m/s)")
    ax[0].legend(loc="best")
    ax[0].grid(alpha=0.3)
    ax[0].set_title(f"Đáp ứng vòng kín PID — {args.csv}")

    ax[1].plot(t, d["pwm_L"], lw=1.3, label="PWM trái")
    ax[1].plot(t, d["pwm_R"], lw=1.3, label="PWM phải")
    ax[1].axhline(PWM_MAX, color="r", ls=":", lw=1)
    ax[1].axhline(-PWM_MAX, color="r", ls=":", lw=1)
    ax[1].set_ylabel("PWM")
    ax[1].legend(loc="best")
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, [s - v for s, v in zip(d["vspL"], d["vL"])], lw=1.3, label="e trái")
    ax[2].plot(t, [s - v for s, v in zip(d["vspR"], d["vR"])], lw=1.3, label="e phải")
    ax[2].axhline(0, color="k", lw=0.8)
    ax[2].set_ylabel("sai số (m/s)")
    ax[2].set_xlabel("thời gian (s)")
    ax[2].legend(loc="best")
    ax[2].grid(alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()