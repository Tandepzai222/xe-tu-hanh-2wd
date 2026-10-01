"""
_console.py — Ép cửa ra console sang UTF-8 để in được tiếng Việt trên Windows.

VẤN ĐỀ
------
Trên Windows, `sys.stdout` mặc định dùng bảng mã **cp1252** (hoặc cp437).
Các ký tự tiếng Việt như `ộ`, `ơ`, `ư`, `đ`, `ạ` KHÔNG có trong cp1252.
Hệ quả: mọi `print("... tiếng Việt ...")` sẽ ném

    UnicodeEncodeError: 'charmap' codec can't encode character '\\u1ed1'

và script chết ngay giữa đường — kể cả `--help`, vì argparse cũng in tài liệu
trợ giúp có dấu tiếng Việt.

GIẢI PHÁP (3 lớp, luỹ tiến)
--------------------------
  1. Đổi code page của console Windows sang 65001 (UTF-8) để terminal
     DIỄN GIẢI đúng các byte UTF-8.
  2. `stream.reconfigure(encoding="utf-8")` để Python MÃ HOÁ ra UTF-8.
  3. `errors="replace"` làm lưới an toàn cuối cùng: nếu vẫn có ký tự không
     mã hoá được thì in `?` chứ **KHÔNG BAO GIỜ crash**.

Cách dùng trong mỗi script
--------------------------
    from _console import setup_console
    setup_console()          # gọi NGAY sau import, TRƯỚC argparse và mọi print
"""

import sys


def setup_console():
    """Ép stdout/stderr sang UTF-8. An toàn khi gọi nhiều lần."""

    # ---- Lớp 1: code page console Windows ----
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleOutputCP(65001)   # UTF-8
            kernel32.SetConsoleCP(65001)
        except Exception:
            # Không có console thật (IDE, pipe, dịch vụ) → bỏ qua, không sao.
            pass

    # ---- Lớp 2 + 3: mã hoá UTF-8, có lưới an toàn ----
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            # Python < 3.7 hoặc stream bị thay bằng đối tượng lạ.
            pass

    # ---- Biến môi trường cho tiến trình con (nếu script gọi subprocess) ----
    try:
        import os
        os.environ.setdefault("PYTHONIOENCODING", "utf-8")
    except Exception:
        pass


# Tự chạy khi được import — để không script nào quên gọi.
setup_console()
