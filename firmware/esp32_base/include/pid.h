/**
 * pid.h — bộ PID rời rạc dùng cho vòng điều khiển vận tốc bánh xe.
 *
 * Đặc điểm (xem docs/03 §4 để hiểu lý do từng lựa chọn):
 *   • Dạng vị trí (position form) — dễ hiểu, hỗ trợ bumpless transfer.
 *   • Đạo hàm theo ĐO LƯỜNG (derivative-on-measurement) → không bị setpoint kick.
 *   • Lọc thông thấp bậc 1 cho khâu D → không khuếch đại nhiễu encoder.
 *   • Anti-windup 3 chế độ, chọn bằng PID_ANTIWINDUP_MODE trong config.h.
 *   • Chặn biên khâu I (iMin/iMax) — lớp bảo vệ thứ hai.
 *   • Feedforward cộng thêm ngoài vòng kín (bù vùng chết của động cơ).
 */
#pragma once
#include <Arduino.h>
#include "config.h"

class PID {
public:
    PID();

    void configure(float kp, float ki, float kd,
                   float outMin, float outMax,
                   float iMin,   float iMax,
                   float dFilterTau);

    void setGains(float kp, float ki, float kd);
    void setKp(float kp) { kp_ = kp; }
    void setKi(float ki) { ki_ = ki; }
    void setKd(float kd) { kd_ = kd; }

    /**
     * Tính ngõ ra.
     * @param sp    setpoint (m/s)
     * @param meas  giá trị đo (m/s)
     * @param dt    chu kỳ lấy mẫu THỰC ĐO (s) — không dùng hằng số giả định
     * @param ff    feedforward (đơn vị ngõ ra, ví dụ PWM chuẩn hoá)
     */
    float update(float sp, float meas, float dt, float ff = 0.0f);

    void reset();
    /** Đặt lại khâu I về một giá trị (dùng cho bumpless transfer). */
    void setIntegral(float i) { integ_ = constrain(i, iMin_, iMax_); }

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }
    float integral() const { return integ_; }
    float lastOutput() const { return lastOut_; }
    float lastP() const { return lastP_; }
    float lastI() const { return lastI_; }
    float lastD() const { return lastD_; }
    float lastFF() const { return lastFF_; }
    bool  saturated() const { return saturated_; }

private:
    float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f;
    float outMin_ = -1.0f, outMax_ = 1.0f;
    float iMin_ = -1.0f, iMax_ = 1.0f;
    float dTau_ = 0.02f;

    float integ_    = 0.0f;
    float prevMeas_ = 0.0f;
    float dFilt_    = 0.0f;
    bool  first_    = true;

    float lastOut_ = 0.0f;
    float lastP_ = 0.0f, lastI_ = 0.0f, lastD_ = 0.0f, lastFF_ = 0.0f;
    bool  saturated_ = false;
};
