/**
 * imu.h — driver I²C thô cho MPU-6065 (họ MPU-60x0 / MPU-6500 / MPU-9250).
 *
 * Thiết kế: tách DRIVER (đọc thanh ghi) khỏi LOGIC (lọc, tích phân yaw).
 * Nếu bạn đổi sang ICM-20948 / ICM-42688-P, chỉ cần viết lại phần driver
 * và giữ nguyên interface Imu::poll() / Imu::data().
 *
 * Cảnh báo: KHÔNG có magnetometer → yaw tích phân sẽ TRÔI.
 * Xem phân tích định lượng ở docs/02 §5.3 và cách khắc phục bằng EKF ở docs/05 §6.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

struct ImuData {
    float ax, ay, az;        // gia tốc (g)
    float gx, gy, gz;        // vận tốc góc (rad/s), ĐÃ trừ bias
    float tempC;             // nhiệt độ (°C)
    float roll, pitch;       // rad, từ complementary filter
    float yaw;               // rad, tích phân gyro, wrap về (−π, π]
};

class Imu {
public:
    Imu() {}

    /** Khởi tạo I²C và cấu hình thanh ghi. Trả về false nếu không tìm thấy chip. */
    bool begin();

    /**
     * Hiệu chuẩn bias gyro. ROBOT PHẢI ĐỨNG YÊN HOÀN TOÀN.
     * Thực hiện 2 lượt: lượt 1 tính mean/std, lượt 2 loại outlier > 3σ.
     * In kết quả ra Serial.
     */
    bool calibrateGyro(uint16_t samples);

    /** Đọc mẫu mới và cập nhật bộ lọc. dt = thời gian từ lần poll trước (s). */
    void poll(float dt);

    const ImuData& data() const { return d_; }

    void  resetYaw(float y = 0.0f) { d_.yaw = y; yawTotal_ = y; }
    float yawTotal() const { return yawTotal_; }   // tích luỹ không wrap (cho hiệu chuẩn)

    bool     ok() const { return ok_; }
    uint8_t  whoAmI() const { return whoAmI_; }
    bool     calibrated() const { return calibrated_; }
    float    biasX() const { return biasX_; }
    float    biasY() const { return biasY_; }
    float    biasZ() const { return biasZ_; }
    float    stdZ()  const { return stdZ_; }

private:
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRegs(uint8_t reg, uint8_t* buf, size_t n);
    bool readGyroZRaw(int16_t& raw);

    ImuData d_ = {};
    bool    ok_         = false;
    bool    calibrated_ = false;
    uint8_t whoAmI_     = 0;

    float biasX_ = 0.0f, biasY_ = 0.0f, biasZ_ = 0.0f;
    float stdZ_  = 0.0f;

    float prevGz_  = 0.0f;
    bool  firstPoll_ = true;
    float yawTotal_  = 0.0f;
};
