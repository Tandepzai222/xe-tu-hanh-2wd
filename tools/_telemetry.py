"""
_telemetry.py — Đọc file CSV do firmware ESP32 xuất ra.

ĐỊNH DẠNG CHUẨN (lệnh `plot <hz>` trên firmware):
    t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R

VẤN ĐỀ KHI ĐỌC FILE NÀY
-----------------------
Người dùng copy nguyên khối output từ Serial Monitor, nên file thường có lẫn:
    • dòng tiêu đề CSV        →  "t_ms,md,vspL,vL,..."
    • dòng nhắc lệnh          →  "> plot 100"
    • thông báo của firmware  →  "[STEP] Ket thuc."
    • dòng trống
    • dòng có khoảng trắng đầu (một số terminal chèn vào khi copy)
    • tiêu đề có khoảng trắng quanh tên cột: "t_ms, md, vL, ..."

Hàm `doc_dong_csv()` lọc đúng: tìm dòng tiêu đề trước, giữ nó lại, rồi chỉ
lấy các dòng dữ liệu phía sau. Hàm `doc_csv()` chuẩn hoá tên cột (bỏ space
+ BOM) TRƯỚC khi đọc, tránh KeyError khi header có space.
"""

import csv


def chuan_hoa_ten_cot(ten):
    """Bỏ khoảng trắng và BOM ở một tên cột."""
    return (ten or "").strip().lstrip("\ufeff")


def _la_dong_du_lieu(ln):
    """True nếu dòng (sau khi bỏ space đầu) bắt đầu bằng chữ số."""
    # SỬA: dùng lstrip() để không loại oan dòng có space đầu
    return ln.lstrip()[:1].isdigit()


def doc_dong_csv(path):
    """
    Trả về danh sách các dòng CSV hợp lệ (đã gồm dòng tiêu đề).

    Nếu file KHÔNG có dòng tiêu đề, trả về các dòng bắt đầu bằng chữ số và
    người gọi phải tự đặt tên cột theo thứ tự.
    """
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        raw = f.readlines()

    # ---- Tìm dòng tiêu đề: dòng đầu tiên chứa "t_ms" ----
    header_idx = None
    for i, ln in enumerate(raw):
        if "t_ms" in ln:
            header_idx = i
            break

    if header_idx is None:
        # Không có tiêu đề → giữ các dòng dữ liệu
        return [ln for ln in raw if _la_dong_du_lieu(ln)]

    # Giữ tiêu đề + mọi dòng dữ liệu phía sau (bỏ qua prompt, thông báo, dòng trống)
    return [raw[header_idx]] + [
        ln for ln in raw[header_idx + 1:] if _la_dong_du_lieu(ln)
    ]


def doc_csv(path, cot_can):
    """
    Đọc CSV và trả về list các dict, CHỈ giữ các cột trong `cot_can`.

    Tự động bỏ khoảng trắng/BOM ở tên cột và bỏ các dòng thiếu/không phải số.

    Raises
    ------
    SystemExit nếu file không có dữ liệu hợp lệ hoặc thiếu cột bắt buộc.
    """
    lines = doc_dong_csv(path)
    if not lines:
        raise SystemExit(
            f"LỖI: không tìm thấy dòng dữ liệu nào trong '{path}'.\n"
            f"     Hãy bật `plot 100` trên firmware rồi copy output vào file."
        )

    reader = csv.DictReader(lines)

    # SỬA: chuẩn hoá fieldnames NGAY khi tạo DictReader. Nếu header có space
    # (ví dụ "t_ms, md, vL"), DictReader sẽ tạo key ' md', ' vL'... Khi đó
    # r['md'] sẽ ném KeyError — mà `except (TypeError, ValueError)` không bắt.
    reader.fieldnames = [chuan_hoa_ten_cot(c) for c in (reader.fieldnames or [])]
    cols = list(reader.fieldnames)

    thieu = [c for c in cot_can if c not in cols]
    if thieu:
        raise SystemExit(
            f"LỖI: file '{path}' thiếu cột {thieu}.\n"
            f"     Cột tìm thấy: {cols}\n"
            f"     Cần định dạng: t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R\n"
            f"     (xuất bằng lệnh `plot 100` trên firmware)"
        )

    rows = []
    for r in reader:
        try:
            # SỬA: bắt thêm KeyError để một dòng lạ không làm chết cả tool
            rows.append({c: float(r[c]) for c in cot_can})
        except (TypeError, ValueError, KeyError):
            continue          # bỏ dòng rác

    if len(rows) < 20:
        raise SystemExit(
            f"LỖI: chỉ có {len(rows)} mẫu hợp lệ trong '{path}' — quá ít để phân tích.\n"
            f"     Cần ít nhất 20 mẫu. Thử ghi lâu hơn: `mode step 400 3000`."
        )
    return rows


def lay_cot(rows, ten):
    """Lấy một cột dưới dạng list số."""
    return [r[ten] for r in rows]