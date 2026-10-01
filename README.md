# ĐỒ ÁN XE TỰ HÀNH 2 BÁNH VI SAI (Differential Drive) — ESP32 → ROS 2 → AI

> Bộ tài liệu giảng dạy + mã nguồn đầy đủ.
> Người soạn: giảng viên hướng dẫn. Người thực hiện: sinh viên.

---

## 0. Bạn đang có gì và sẽ đi tới đâu

**Phần cứng hiện tại (Phase 1 — "robot chạy được")**

| Thành phần | Thông số | Vai trò |
|---|---|---|
| Khung robot tròn | 2 bánh động + 1 bánh phụ (caster) | Cấu hình vi sai |
| Động cơ | JGA25-370 12V, 130 RPM (trục ra), hộp giảm tốc 1:47.5 | Truyền động |
| Encoder | Hall 2 kênh, N = **1975.8** xung/vòng bánh (✅ **đã đo thực nghiệm**) | Đo quãng đường |
| Bánh xe | Đường kính D = 65 mm | — |
| Khoảng cách 2 bánh | L = 185 mm | Thông số động học |
| Driver | TB6612FNG (2 kênh H-bridge) | Điều khiển động cơ |
| Vi điều khiển | ESP32 (WROOM-32) | Điều khiển + WiFi |
| IMU | MPU-6065 (họ MPU-60x0/6500, I²C) | Gyro/Accel → góc hướng |
| Nguồn | Pin 3S LiPo ~11.1V (12.6V khi đầy) | Năng lượng |
| Hạ áp | Buck 12V → 5V | Nuôi ESP32 |

**Nâng cấp (Phase 2/3)**

| Thành phần | Vai trò |
|---|---|
| Raspberry Pi 4 | Chạy ROS 2 onboard: SLAM, Nav2, camera, AI |
| LiDAR 2D | SLAM + định vị + tránh vật cản |
| Module camera OV5640 (CSI-2) | Thị giác, AI (nhận diện làn, vật thể, người) |

**Kiến trúc mục tiêu**

```
┌─────────────────────────── ROBOT ───────────────────────────┐
│                                                             │
│   [Encoder L/R]──┐                        ┌──[TB6612]──[M L] │
│                  ├──► ESP32 (100 Hz) ─────┤                 │
│   [MPU-6065]─────┘    • PID vòng vận tốc  └──[TB6612]──[M R] │
│                       • Odometry vi sai                      │
│                       • An toàn/watchdog                     │
│                            │ WiFi UDP (JSON)                 │
│                            ▼                                 │
│                    Raspberry Pi 4 (ROS 2 Humble)             │
│                    • micro-ROS / udp_bridge                  │
│                    • robot_localization (EKF)                │
│                    • slam_toolbox → Nav2                     │
│                    • camera_ros → AI node                    │
└──────────────────────────────┬──────────────────────────────┘
                               │ WiFi / SSH / RViz2
                               ▼
                    Laptop (hiển thị, giám sát, ghi log)
```

**Nguyên tắc thiết kế quan trọng nhất của đồ án:**
> ESP32 **không** biết gì về SLAM/Nav2. Nó chỉ làm đúng 2 việc: (1) điều khiển vận tốc 2 bánh thật chính xác, (2) báo cáo odometry + IMU thật trung thực.
> Mọi thứ "thông minh" nằm ở ROS 2. Đây là cách phân chia trách nhiệm giúp đồ án dễ debug, dễ mở rộng, và dễ viết báo cáo.

---

## 1. Bản đồ tài liệu — đọc theo thứ tự này

| # | File | Nội dung | Khi nào đọc |
|---|---|---|---|
| 1 | [`docs/01_phan-cung-va-dau-noi.md`](docs/01_phan-cung-va-dau-noi.md) | Pinout ESP32, sơ đồ đấu dây, ngân sách nguồn, chống nhiễu, BOM, an toàn pin LiPo | Trước khi hàn |
| 2 | [`docs/02_toan-hoc-odometry-va-IMU.md`](docs/02_toan-hoc-odometry-va-IMU.md) | **Toàn bộ toán học**: độ phân giải encoder, động học vi sai, tích phân odometry, hiệu chuẩn, xử lý IMU, lan truyền sai số | Chương 2–3 của báo cáo |
| 3 | [`docs/03_dong-co-va-dieu-khien-PID.md`](docs/03_dong-co-va-dieu-khien-PID.md) | **Toàn bộ lý thuyết PID**: mô hình động cơ DC, dạng rời rạc, anti-windup, lọc đạo hàm, feedforward, 3 phương pháp tuning (IMC / relay / ZN) | Chương 4 của báo cáo |
| 4 | [`docs/04_firmware-esp32.md`](docs/04_firmware-esp32.md) | Kiến trúc firmware, luồng RTOS, các chế độ test, quy trình nạp & hiệu chuẩn | Khi bắt đầu code |
| 5 | [`docs/05_giao-thuc-va-ros2.md`](docs/05_giao-thuc-va-ros2.md) | Đặc tả giao thức UDP, ánh xạ topic ROS 2, bridge node, URDF/TF/EKF, SLAM, Nav2 | Phase 2 |
| 6 | [`docs/06_lo-trinh-nang-cap-ai.md`](docs/06_lo-trinh-nang-cap-ai.md) | LiDAR, Pi 4, OV5640, pipeline AI, kiến trúc lai | Phase 3 |
| 7 | [`docs/07_checklist-cong-viec.md`](docs/07_checklist-cong-viec.md) | **CHECKLIST công việc** 11 giai đoạn, có tiêu chí nghiệm thu từng bước | Dùng hằng ngày |

**Mã nguồn — bản đồ file**

```
firmware/esp32_base/                 PlatformIO, ESP32 DevKit V1
├── platformio.ini                   espressif32@6.9.0 (Arduino core 2.0.17)
├── include/
│   ├── config.h                     ⭐ TOÀN BỘ tham số nằm ở đây — sửa file này trước
│   ├── encoder.h   src/encoder.cpp  Đọc quadrature ×4 bằng PCNT phần cứng
│   ├── motor.h     src/motor.cpp    Điều khiển TB6612 qua LEDC 20 kHz / 10 bit
│   ├── pid.h       src/pid.cpp      PID + lọc đạo hàm + 3 chế độ anti-windup
│   ├── imu.h       src/imu.cpp      MPU-60x0/6500: I²C, hiệu chuẩn bias, lọc bù
│   ├── safety.h    src/safety.cpp   Watchdog lệnh, kẹt bánh, điện áp pin
│   ├── robot.h     src/robot.cpp    ⭐ Lớp trung tâm: 10 chế độ, odometry, telemetry
│   ├── comms.h     src/comms.cpp    WiFi UDP + console Serial + NVS
│   └── main.cpp                     Tạo controlTask (core 1, prio 10) + loopTask
│
ros2_ws/src/xe_tu_hanh/               package ament_python, ROS 2 Humble
├── package.xml  setup.py  setup.cfg
├── xe_tu_hanh/udp_bridge.py         ⭐ Cầu nối UDP ↔ ROS 2: /odom, /imu/data,
│                                       /battery_state, /cmd_vel, TF, chẩn đoán
├── urdf/xe_tu_hanh.urdf.xacro       Mô hình hình học (số phải khớp config.h)
├── config/ekf.yaml                  robot_localization EKF (two_d_mode)
├── config/slam_toolbox.yaml         SLAM 2D online asynchronous
├── config/nav2_params.yaml          Nav2: controller, costmap, planner, smoother
└── launch/robot.launch.py  slam.launch.py  nav2.launch.py
│
tools/                               Python 3, chỉ cần thư viện chuẩn (+ matplotlib)
├── wheel_calib_calc.py              Tính N, d_pc, L từ số liệu đo
├── pid_step_analysis.py             Nhận dạng FOPDT + tính gain IMC-PI
├── plot_telemetry.py                Đánh giá chất lượng PID + vẽ đồ thị
└── udp_test.py                      ping / mon / drive / square — test UDP
```

---

## 2. Cài đặt nhanh (Quickstart)

### 2.1. Nạp firmware

```bash
# Cài PlatformIO (nếu chưa có)
pip install -U platformio

# Trong thư mục firmware
cd firmware/esp32_base
pio run                 # build
pio run -t upload       # nạp
pio device monitor -b 115200
```

Trước khi build, **bắt buộc sửa** `include/config.h`:
- `WIFI_SSID`, `WIFI_PASS`
- `LAPTOP_IP` (địa chỉ IP laptop chạy ROS 2)
- `WHEEL_TRACK_M` — đo bằng thước cặp, tâm-tâm 2 bánh (hiện đặt 185 mm)
- `COUNTS_PER_WHEEL_REV` — đã đặt `1975.8f` theo số đo của bạn ✅
- `USE_MEASURED_DPC` — để `0` nếu đo N bằng quay tay; đổi sang `1` nếu phép
  thử 3 m cho thấy robot đi hụt/dư quá 2%

### 2.2. Chạy thử ngay sau khi nạp (không cần ROS 2)

Firmware có sẵn chế độ chạy độc lập qua Serial Monitor 115200 baud:

```
> status                 # kiểm tra HZ ≈ 100, JIT < 200 µs, ERR = 0x0000
> mode duty 300 300      # chạy open-loop 30% cả 2 bánh (kê bánh lên giá!)
> enc                    # kiểm tra 2 encoder đều tăng, không bánh nào âm
> mode wheel 0.2 0.2     # PID vòng vận tốc: 0.2 m/s cả 2 bánh
> plot 100               # xuất CSV để vẽ đồ thị
> mode step 400 3000     # test step response -> nhận dạng động cơ
> mode heading 0.2       # chạy thẳng 0.2 m/s, giữ hướng bằng IMU
> mode goto 1.0 0.5      # tự hành tới điểm (1.0 m, 0.5 m)
> stop / estop          # dừng / dừng khẩn
> help                   # in toàn bộ lệnh
```

### 2.3. Hiệu chuẩn (làm trước khi tune PID)

```
> cal imu                # robot đứng yên 2 s
> cal deadzone l         # tìm vùng chết PWM bánh trái (bánh quay tự do)
> cal deadzone r
> cal enc                # rồi ĐẨY TAY robot đúng 3.000 m, sau đó gõ `enc`
> cal spin 5             # quay tại chỗ 5 vòng -> firmware tự tính L mới
> cal batt 12.35         # nhập điện áp đo bằng đồng hồ -> ra BATT_CAL_GAIN
```

Hoặc dùng script Python để tính toán và vẽ đồ thị:

```bash
python tools/wheel_calib_calc.py spin --odom-deg 1794.3 --imu-deg 1801.1
python tools/pid_step_analysis.py step_400.csv --wheel l --plot
python tools/plot_telemetry.py telemetry.csv --target 0.2
```

### 2.4. Kiểm tra WiFi trước khi cài ROS 2

```bash
python tools/udp_test.py ping  --ip 192.168.1.50     # đo RTT & mất gói
python tools/udp_test.py mon   --ip 192.168.1.50     # xem telemetry
python tools/udp_test.py drive --ip 192.168.1.50 --v 0.15 --secs 5
```

### 2.5. Chạy ROS 2 (Phase 2)

```bash
mkdir -p ~/ros2_ws/src && cp -r ros2_ws/src/xe_tu_hanh ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch xe_tu_hanh robot.launch.py esp32_ip:=192.168.1.50 use_rviz:=true
```

---

## 3. Các con số "chốt" của đồ án (để bạn kiểm tra chéo)

| Đại lượng | Ký hiệu | Giá trị | Ghi chú |
|---|---|---|---|
| Chu vi bánh | C | 204.204 mm | `C = πD` |
| Quãng đường / 1 xung | `d_pc` | **0.10335 mm** | `C / N` |
| Số xung / 1 mét | — | 9675.7 | nghịch đảo |
| Vận tốc tối đa (không tải) | `v_max` | **0.442 m/s** (1.59 km/h) | `130rpm × C / 60` |
| Vận tốc thiết kế | `v_cruise` | 0.20 m/s | ~45% tải tối đa |
| Tốc độ góc tối đa khi quay tại chỗ | `ω_max` | 4.78 rad/s (274 °/s) | `2v_max / L` |
| Bán kính quay nhỏ nhất | `R_min` | 92.5 mm | `L/2` |
| Độ phân giải góc của odometry | `Δθ_min` | 0.032° | `d_pc / L` |
| Tần số vòng PID | `f_c` | 100 Hz | `Ts = 10 ms` (task RTOS core 1) |

> ⚠️ `N = 1975.8` **không** chia hết cho `4 × 47.5 = 190` (kết quả 10.4) → **tỉ số truyền 1:47.5 trên nhãn là không chính xác**. Con số `1975.8` là **số đo thực nghiệm của bạn** (quay tay nhiều vòng rồi lấy trung bình) nên **nó có thẩm quyền cao nhất** — giữ nguyên, không sửa.
>
> Phân tích: `1975.8 ≈ 1976 = 13 × 38 × 4` → hộp giảm tốc thực gần như chắc chắn là **1:38** với encoder **13 PPR**. Sai khác `1975.8` vs `1976` chỉ **0.01%** (0.1 mm mỗi mét) → không cần xử lý.
>
> ⚠️ **Việc còn lại bắt buộc:** vì bạn đo bằng quay tay, con số này **chưa** phản ánh bán kính lăn thực của lốp khi chịu tải. Phải kiểm tra chéo bằng phép thử 3 m (Doc 02 §1.2.1 + §7.1). Sai 1% ở `d_pc` → sai 10 mm mỗi mét → robot lệch khỏi bản đồ SLAM.

---

## 4. Lộ trình tổng thể (tóm tắt — chi tiết ở Doc 07)

```
GĐ 0  Chuẩn bị: dụng cụ, kiến thức, môi trường      →  "Sẵn sàng"
GĐ 1  Lắp cơ khí + đi dây + đo nguồn                →  "Cấp nguồn không cháy"
GĐ 2  Firmware nền: encoder + motor + serial        →  "Đọc được xung & quay được bánh"
GĐ 3  Hiệu chuẩn: deadzone, d_pc, L, IMU bias       →  "Số liệu đáng tin"
GĐ 4  PID vòng vận tốc từng bánh                    →  "Sai số vận tốc < 5%"
GĐ 5  Odometry + IMU + đi thẳng/giữ hướng           →  "Đi thẳng 3 m lệch < 10 cm"
GĐ 6  WiFi + giao thức UDP                          →  "Laptop nhận telemetry 50 Hz"
GĐ 7  ROS 2: bridge, URDF/TF, EKF, teleop, RViz     →  "RViz vẽ đúng quỹ đạo"
GĐ 8  LiDAR + SLAM Toolbox                          →  "Có bản đồ + định vị"
GĐ 9  Nav2                                          →  "Robot tự tới đích tránh vật cản"
GĐ 10 Pi 4 onboard + OV5640 + AI                    →  "Nhận diện & bám mục tiêu"
GĐ 11 Đóng gói: báo cáo, video, tài liệu            →  "Bảo vệ"
```

---

## 5. Quy ước ký hiệu & hệ trục

**Hệ trục theo chuẩn ROS (REP-103):**

- `x` — hướng tiến của robot (mét)
- `y` — sang **trái** của robot (mét)
- `z` — hướng lên
- `θ` (yaw) — dương khi quay **ngược kim đồng hồ** (CCW), tính bằng radian
- Vận tốc: `v` (m/s) dọc `x`, `ω` (rad/s) quanh `z`

Quy ước bánh: bánh **L (Left)** = bánh trái, bánh **R (Right)** = bánh phải, khi đứng sau robot nhìn về phía trước.

**Ký hiệu toán học dùng xuyên suốt:**

| Ký hiệu | Ý nghĩa | Đơn vị |
|---|---|---|
| `N` | số xung encoder trên 1 vòng **bánh** | xung/vòng |
| `d_pc` | quãng đường đi được khi encoder tăng 1 xung | m/xung |
| `D`, `R` | đường kính / bán kính bánh | m |
| `L` | khoảng cách giữa 2 tâm bánh (track width) | m |
| `v_L`, `v_R` | vận tốc dài của bánh trái/phải | m/s |
| `v`, `ω` | vận tốc dài và vận tốc góc của thân robot | m/s, rad/s |
| `Ts` | chu kỳ lấy mẫu của vòng điều khiển | s |
| `u` | tín hiệu điều khiển (PWM chuẩn hóa) | −1…+1 |

---

## 6. Cảnh báo an toàn (đọc trước khi làm)

1. **Pin LiPo 3S:** không để xả dưới 3.0 V/cell (9.0 V pack). Dùng buzzer báo pin hoặc đo qua ADC trong firmware (đã tích hợp). Sạc bằng sạc cân bằng, không sạc qua đêm không giám sát.
2. **Không bao giờ để robot tự chạy khi đang nạp firmware** — luôn kê bánh lên giá (test stand) ở GĐ 2–4.
3. **Nối mass (GND) chung** giữa pin, buck, TB6612 và ESP32. Thiếu GND chung → PWM chạy sai, ESP32 reset ngẫu nhiên.
4. **Cầu chì** 5 A trên đường `+` của pin.
5. **Công tắc nguồn** ở vị trí với tới được. Khi test PID, tay luôn để trên công tắc.
6. GPIO ESP32 là **3.3 V** — không cấp 5 V vào chân GPIO.

---

## 7. Trạng thái bộ tài liệu

| Hạng mục | Trạng thái | Ghi chú |
|---|---|---|
| Doc 01 — Phần cứng & đấu nối | ✅ đầy đủ | Pinout, ngân sách nguồn, BOM, 7 bước smoke test |
| Doc 02 — Toán học odometry & IMU | ✅ đầy đủ | Có chứng minh công thức, 5 quy trình hiệu chuẩn, bảng số liệu mẫu |
| Doc 03 — Động cơ & PID | ✅ đầy đủ | Mô hình FOPDT, 3 cách tuning, quy trình 9 bước, bảng lỗi thường gặp |
| Doc 04 — Firmware ESP32 | ✅ đầy đủ | Khớp 1-1 với code trong `firmware/` |
| Doc 05 — Giao thức & ROS 2 | ✅ đầy đủ | Đặc tả UDP, TF, EKF, SLAM, Nav2 |
| Doc 06 — Lộ trình AI | ✅ đầy đủ | Mức kiến trúc |
| Doc 07 — Checklist công việc | ✅ đầy đủ | 11 giai đoạn, ~62 buổi, có tiêu chí nghiệm thu |
| Firmware ESP32 | ✅ **code hoàn chỉnh** | 9 module, ~2 400 dòng. Cần build thử lần đầu trên máy bạn |
| Tools Python | ✅ **4 script** | Không cần thư viện ngoài (matplotlib là tuỳ chọn) |
| ROS 2 package | ✅ **code khung chạy được** | `udp_bridge.py` đầy đủ; cần build trên Pi/Ubuntu để xác nhận |
| SLAM / Nav2 config | ⚠️ mẫu tham số | **Phải chỉnh** `max_laser_range`, `lidar_x/y/z`, `robot_radius` theo phần cứng thực |
| AI (OV5640 + Pi 4) | ⚠️ kiến trúc + lộ trình | Code khi tới GĐ 10 |

### Những việc **bạn** phải làm tiếp (không phải mình bỏ dở)

| # | Việc | Vì sao không làm được từ xa |
|---|---|---|
| 1 | Build thử `pio run` và sửa lỗi biên dịch nếu có | Cần toolchain trên máy bạn |
| 2 | Phép thử 3 m để xác nhận `d_pc` | Cần robot thật |
| 3 | Đo `L` bằng thước cặp | Cần robot thật |
| 4 | `cal deadzone l/r`, `cal imu` | Cần robot thật |
| 5 | Nhận dạng động cơ (`mode step`) và nạp gain IMC | Cần số liệu thật |
| 6 | Đo `lidar_x/y/z` bằng thước khi gắn LiDAR | Cần phần cứng |
| 7 | Kiểm tra `WHO_AM_I` của MPU-6065 qua `imu` | Cần phần cứng (firmware tự nhận dạng) |

Những chỗ đánh dấu ⚠️ là **bạn phải làm tiếp**, không phải mình bỏ dở — vì chúng phụ thuộc vào phần cứng bạn chưa mua.
