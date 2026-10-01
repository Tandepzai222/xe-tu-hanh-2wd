/**
 * motor.h — điều khiển TB6612FNG qua LEDC.
 *
 * Bảng chân lý TB6612FNG (xem docs/01 §3.2):
 *   IN1=H IN2=L PWM=H → quay thuận
 *   IN1=L IN2=H PWM=H → quay nghịch
 *   IN1=L IN2=L        → coast (trôi tự do)
 *   IN1=H IN2=H        → short brake
 *   PWM=0 với IN1≠IN2  → short brake  (⚠️ KHÔNG phải coast!)
 *
 * Quy ước dấu: u > 0 là bánh quay theo chiều TIẾN của robot.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

class Motor {
public:
    Motor(uint8_t pinPWM, uint8_t pinIN1, uint8_t pinIN2, uint8_t ledcCh, bool invert);

    void begin();

    /** u ∈ [−1, +1]. Tự áp dụng cờ đảo chiều. */
    void setDuty(float u);

    /** Hãm ngắn mạch (dừng nhanh, motor sinh dòng ngược). */
    void brake();

    /** Trôi tự do (đầu ra trở kháng cao). */
    void coast();

    /** Cho phép / khoá đầu ra (không phụ thuộc STBY của TB6612). */
    void setEnabled(bool en) { enabled_ = en; if (!en) coast(); }
    bool enabled() const { return enabled_; }

    float lastDuty() const { return lastU_; }

private:
    uint8_t pinPWM_, pinIN1_, pinIN2_, ledcCh_;
    bool    invert_;
    bool    enabled_ = false;
    float   lastU_   = 0.0f;
};
