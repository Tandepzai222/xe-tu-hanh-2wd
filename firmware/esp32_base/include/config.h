/**
 * ============================================================================
 *  config.h — TOÀN BỘ THAM SỐ CỦA ĐỒ ÁN NẰM Ở ĐÂY
 * ============================================================================
 *  Nguyên tắc: KHÔNG hard-code bất kỳ con số nào trong các file .cpp khác.
 *  Muốn tune → chỉ sửa file này (hoặc dùng lệnh `cfg` / `pid` qua Serial/UDP).
 *
 *  Các giá trị đánh dấu [ĐO] là giá trị ĐIỂN HÌNH, bạn PHẢI thay bằng số đo
 *  thực tế của mình theo quy trình trong docs/02_toan-hoc-odometry-va-IMU.md §7
 *  và docs/03_dong-co-va-dieu-khien-PID.md §5.
 * ============================================================================
 */
#pragma once
#include <Arduino.h>

// ============================================================================
//  1. ĐỊNH DANH
// ============================================================================
#define FW_VERSION                  "0.9.0"

// ============================================================================
//  2. WIFI  (⚠️ BẮT BUỘC SỬA TRƯỚC KHI BUILD)
// ============================================================================
#define WIFI_SSID                   "Cong Khanh."
#define WIFI_PASS                   "hoanglong12"
#define LAPTOP_IP                   "192.168.1.12"   // IP máy chạy ROS 2
#define UDP_PORT_TELEM              8888              // ESP32 → laptop
#define UDP_PORT_CMD                8889              // laptop → ESP32
#define WIFI_CONNECT_TIMEOUT_MS     15000
#define WIFI_RETRY_INTERVAL_MS      5000
#define UDP_BROADCAST_FALLBACK      0                 // 1 = gửi broadcast nếu chưa biết IP

// ============================================================================
//  3. THÔNG SỐ CƠ KHÍ  [ĐO LẠI theo docs/02 §7]
// ============================================================================
#define WHEEL_DIAMETER_M            0.065f      // D = 65 mm
#define WHEEL_RADIUS_M              (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_TRACK_M               0.1718f      // L = khoảng cách 2 TÂM bánh
#define COUNTS_PER_WHEEL_REV        1953.0f     // N — ✅ SỐ ĐO THỰC NGHIỆM của bạn
#define GEAR_RATIO                  47.5f
#define ENCODER_EDGES               4           // quadrature ×4

#define WHEEL_CIRCUMFERENCE_M       (PI * WHEEL_DIAMETER_M)

// ---------------------------------------------------------------------------
//  CÁCH TÍNH d_pc — CHỌN ĐÚNG theo phương pháp bạn đã đo N   (docs/02 §1.2.1)
// ---------------------------------------------------------------------------
//  0 = N đo bằng cách QUAY TAY ĐÚNG 1 VÒNG BÁNH (đếm xung/vòng)
//      → d_pc = π·D / N      (giả định bán kính lăn hiệu dụng = D danh định)
//  1 = d_pc đo TRỰC TIẾP bằng cách CHẠY MỘT QUÃNG ĐƯỜNG ĐÃ BIẾT
//      → nhập thẳng MEASURED_DPC_M = S_thực / tổng_xung  (chính xác nhất)
#define USE_MEASURED_DPC            0
#define MEASURED_DPC_M              (2.800f / 26808.0f)   // ← thay bằng số của bạn

#if USE_MEASURED_DPC
  #define DISTANCE_PER_COUNT_M      MEASURED_DPC_M
  #define COUNTS_PER_METER          (1.0f / MEASURED_DPC_M)
#else
  #define DISTANCE_PER_COUNT_M      (WHEEL_CIRCUMFERENCE_M / COUNTS_PER_WHEEL_REV)
  #define COUNTS_PER_METER          (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_M)
#endif

// --- Giới hạn động học suy ra từ cơ khí ---
#define MOTOR_MAX_RPM               130.0f      // không tải, tại 12 V
#define MOTOR_SHAFT_RPM             (MOTOR_MAX_RPM * GEAR_RATIO)
#define WHEEL_MAX_RAD_S             (MOTOR_MAX_RPM * 2.0f * PI / 60.0f)
#define V_MAX_MPS                   (WHEEL_MAX_RAD_S * WHEEL_RADIUS_M)   // ≈ 0.4425
#define W_MAX_RAD_S                 (2.0f * V_MAX_MPS / WHEEL_TRACK_M)   // ≈ 4.784

// ⚠️ Vận tốc có thể ĐIỀU KHIỂN thực tế thấp hơn V_MAX vì feedforward bão hoà
//    (xem docs/03 §7.2). Đây là con số dùng để cấu hình Nav2 max_vel_x.
#define V_SAFE_MPS                  0.30f

// ============================================================================
//  4. BẢN ĐỒ CHÂN — ESP32 DevKit V1 30 chân (ESP32-WROOM-32)
//     Đồng bộ với bảng chân 2026-09-30 (ảnh người dùng gửi)
// ============================================================================
#define PIN_ENC_L_A                 34      // input-only — Kênh A encoder trái (PCNT)
#define PIN_ENC_L_B                 35      // input-only — Kênh B encoder trái
#define PIN_ENC_R_A                 18      // Kênh A encoder phải
#define PIN_ENC_R_B                 19      // Kênh B encoder phải

#define PIN_I2C_SDA                 21      // MPU6050
#define PIN_I2C_SCL                 22      // MPU6050

#define PIN_MOT_L_PWM               16      // LEDC ch 0 — PWM trái (TB6612)
#define PIN_MOT_L_IN1               17      // Chiều quay trái
#define PIN_MOT_L_IN2               4       // Chiều quay trái

#define PIN_MOT_R_PWM               25      // LEDC ch 1 — PWM phải (TB6612)
#define PIN_MOT_R_IN1               26      // Chiều quay phải
#define PIN_MOT_R_IN2               27      // Chiều quay phải

#define PIN_TB_STBY                 23      // Chân standby TB6612 (an toàn khởi động)
#define PIN_BATT_ADC                32      // ADC1_CH4 — đo pin qua cầu phân áp
#define PIN_LED_STATUS              2       // LED onboard (báo WiFi/lỗi)

#define LEDC_CH_L                   0
#define LEDC_CH_R                   1

// --- Đảo chiều logic (đặt 1 nếu bánh quay sai chiều) ---
#define MOTOR_L_INVERT              0
#define MOTOR_R_INVERT              0
#define ENC_L_INVERT                1
#define ENC_R_INVERT                0

// ============================================================================
//  5. PWM
// ============================================================================
//  Giới hạn LEDC: 2^bits × f_PWM ≤ 80 MHz.
//  20 kHz × 2^10 = 20.48 MHz  ✅   (20 kHz × 2^12 = 81.9 MHz ❌)
//  20 kHz nằm trên ngưỡng nghe → động cơ không rít.
#define PWM_FREQ_HZ                 20000
#define PWM_RES_BITS                10
#define PWM_MAX                     ((1 << PWM_RES_BITS) - 1)     // 1023

// ============================================================================
//  6. VÒNG ĐIỀU KHIỂN
// ============================================================================
#define CONTROL_PERIOD_US           10000       // Ts = 10 ms → 100 Hz
#define CONTROL_DT_S                0.010f
#define CONTROL_TASK_CORE           1
#define CONTROL_TASK_PRIO           10
#define CONTROL_TASK_STACK          8192
#define DT_MIN_S                    0.002f      // chặn dưới khi tính dt thực
#define DT_MAX_S                    0.050f      // chặn trên  khi tính dt thực

#define IMU_PERIOD_MS               5           // đọc IMU 200 Hz
#define TELEM_PERIOD_MS             20          // telemetry UDP 50 Hz
#define STATUS_LED_PERIOD_MS        200

// ============================================================================
//  7. LỌC TÍN HIỆU
// ============================================================================
//  Hệ số alpha = tau / (tau + dt).  tau = VEL_FILTER_TAU_S
//  ⚠️ Lọc làm TĂNG độ trễ → giới hạn gain PID (docs/03 §2.3).
#define VEL_FILTER_TAU_S            0.015f

// ============================================================================
//  8. MÔ HÌNH ĐỘNG CƠ + FEEDFORWARD  [ĐO theo docs/03 §5, §7]
// ============================================================================
#define MOTOR_K                     0.37f       // (m/s) trên 1 đơn vị PWM  [ĐO]
#define MOTOR_TAU_S                 0.092f      // hằng số thời gian [s]    [ĐO]
#define MOTOR_TD_S                  0.020f      // thời gian trễ [s]        [ĐO]
#define DZ_FRACTION                 0.14f       // vùng chết (0..1)         [ĐO]
#define DZ_FRACTION_L               DZ_FRACTION
#define DZ_FRACTION_R               DZ_FRACTION
#define FF_GAIN                     1.00f       // hệ số nhân feedforward (tinh chỉnh nhanh)
#define FF_ENABLE                   1
#define V_SP_DEADBAND               0.005f      // dưới ngưỡng này coi như đứng yên

// ============================================================================
//  9. PID VÒNG VẬN TỐC  (khởi điểm IMC với λ = τ — docs/03 §6.1)
// ============================================================================
#define PID_KP_DEFAULT              2.20f
#define PID_KI_DEFAULT              24.00f
#define PID_KD_DEFAULT              0.00f       // τ/td = 4.6 → KHÔNG dùng D

#define PID_D_FILTER_TAU_S          0.020f
#define PID_I_MIN                   (-0.60f)    // chặn biên khâu I — lớp bảo vệ 2
#define PID_I_MAX                   ( 0.60f)
#define PID_OUT_MIN                 (-1.00f)
#define PID_OUT_MAX                 ( 1.00f)

// Chế độ anti-windup: 0 = none, 1 = clamping (khuyến nghị), 2 = back-calculation, 3 = integral separation
#define PID_ANTIWINDUP_MODE         1
#define PID_BACKCALC_TT_S           MOTOR_TAU_S
#define PID_INT_SEP_THRESH          0.10f       // m/s

// ============================================================================
// 10. GIỚI HẠN ĐỘNG HỌC
// ============================================================================
#define ACC_MAX_MPS2                1.00f       // gia tốc dài tối đa
#define ALPHA_MAX_RADPS2            6.00f       // gia tốc góc tối đa
#define SLEW_ENABLE                 1

// ============================================================================
// 11. VÒNG GIỮ HƯỚNG  (ω = Kp·e_θ − Kd·ω_gyro  → "rate feedback")
// ============================================================================
#define HEADING_KP                  2.50f
#define HEADING_KD                  0.35f
#define HEADING_W_MAX               (W_MAX_RAD_S * 0.85f)

// ============================================================================
// 12. GO-TO-GOAL
// ============================================================================
#define GOTO_KDIST                  0.80f
#define GOTO_TOL_M                  0.05f
#define GOTO_V_MAX                  V_SAFE_MPS

// ============================================================================
// 13. AN TOÀN
// ============================================================================
#define CMD_TIMEOUT_MS              300         // watchdog lệnh từ xa
#define CMD_TIMEOUT_ENABLE          1
#define WIFI_TIMEOUT_MS             1000
#define STALL_PWM_THRESH            0.70f
#define STALL_VEL_THRESH            0.02f       // m/s
#define STALL_TIME_MS               500
#define BATT_WARN_V                 10.50f
#define BATT_CRIT_V                 9.90f
#define BATT_DIVIDER_R1_OHM         100000.0f
#define BATT_DIVIDER_R2_OHM         22000.0f
#define BATT_DIVIDER_RATIO          ((BATT_DIVIDER_R1_OHM + BATT_DIVIDER_R2_OHM) / BATT_DIVIDER_R2_OHM)
#define BATT_CAL_GAIN               1.000f      // hiệu chỉnh bằng lệnh `cal batt <V>`
#define BATT_ADC_SAMPLES            16

// ============================================================================
// 14. IMU MPU-60x0 / 6500
// ============================================================================
#define IMU_I2C_ADDR                0x68        // AD0 → GND
#define IMU_I2C_FREQ                400000L
#define IMU_GYRO_FS_SEL             0x08        // ±500 °/s  → 65.5 LSB/(°/s)
#define IMU_GYRO_SENS               65.5f
#define IMU_ACCEL_AFS_SEL           0x00        // ±2 g      → 16384 LSB/g
#define IMU_ACCEL_SENS              16384.0f
#define IMU_DLPF_CFG                0x03        // DLPF 42 Hz
#define IMU_SMPLRT_DIV              4           // 1 kHz / (1+4) = 200 Hz
#define IMU_CALIB_SAMPLES           1000
#define IMU_CALIB_MAX_STD_RADPS     0.02f       // cảnh báo nếu robot đang rung
#define IMU_COMP_ALPHA              0.98f
#define IMU_GZ_SIGN                 (+1.0f)     // đổi thành −1 nếu quay CCW mà gz âm
#define IMU_GX_SIGN                 (+1.0f)
#define IMU_GY_SIGN                 (+1.0f)
#define IMU_AX_SIGN                 (+1.0f)
#define IMU_AY_SIGN                 (+1.0f)
#define IMU_AZ_SIGN                 (+1.0f)

// ============================================================================
// 15. ODOMETRY
// ============================================================================
//  1 = Euler        (đơn giản, sai số O(Δθ²))
//  2 = Mid-point    (khuyến nghị, sai số O(Δθ³))   ← mặc định
//  3 = Exact arc    (chính xác nhất, có điểm kỳ dị khi Δθ→0)
#define ODOM_INTEGRATION            2

// ============================================================================
// 16. NVS (lưu tham số vào flash)
// ============================================================================
#define NVS_NAMESPACE               "robot"
#define CONFIG_VERSION              3

// ============================================================================
// 17. DEBUG
// ============================================================================
#define SERIAL_BAUD                 115200
#define DEBUG_ENABLE                1
#define LOG_LEVEL                   2           // 0=im lặng 1=lỗi 2=thông tin 3=chi tiết
