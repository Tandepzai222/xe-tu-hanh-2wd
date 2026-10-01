#include "motor.h"

Motor::Motor(uint8_t pinPWM, uint8_t pinIN1, uint8_t pinIN2, uint8_t ledcCh, bool invert)
    : pinPWM_(pinPWM), pinIN1_(pinIN1), pinIN2_(pinIN2), ledcCh_(ledcCh), invert_(invert) {}

void Motor::begin() {
    pinMode(pinIN1_, OUTPUT);
    pinMode(pinIN2_, OUTPUT);
    digitalWrite(pinIN1_, LOW);
    digitalWrite(pinIN2_, LOW);

    double actualFreq = ledcSetup(ledcCh_, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pinPWM_, ledcCh_);
    ledcWrite(ledcCh_, 0);

    // Kiểm tra giới hạn LEDC: 2^bits × f phải ≤ 80 MHz, nếu không core sẽ
    // âm thầm giảm độ phân giải.
    if (actualFreq < (double)PWM_FREQ_HZ * 0.99) {
        Serial.printf("[MOTOR] CANH BAO: LEDC tra ve %.1f Hz (yeu cau %d Hz) "
                      "-> do phan giai bi giam. Kiem tra PWM_RES_BITS.\n",
                      actualFreq, PWM_FREQ_HZ);
    }

    lastU_   = 0.0f;
    enabled_ = false;
}

void Motor::setDuty(float u) {
    if (!enabled_) u = 0.0f;
    if (invert_)   u = -u;
    u = constrain(u, -1.0f, 1.0f);

    uint32_t duty = (uint32_t)(fabsf(u) * (float)PWM_MAX + 0.5f);

    if (u > 0.0f) {
        digitalWrite(pinIN1_, HIGH);
        digitalWrite(pinIN2_, LOW);
    } else if (u < 0.0f) {
        digitalWrite(pinIN1_, LOW);
        digitalWrite(pinIN2_, HIGH);
    } else {
        // u == 0 → coast (KHÔNG dùng short brake, để PID tự đưa u về giá trị cần)
        digitalWrite(pinIN1_, LOW);
        digitalWrite(pinIN2_, LOW);
    }

    ledcWrite(ledcCh_, duty);
    lastU_ = u;
}

void Motor::brake() {
    digitalWrite(pinIN1_, HIGH);
    digitalWrite(pinIN2_, HIGH);
    ledcWrite(ledcCh_, 0);
    lastU_ = 0.0f;
}

void Motor::coast() {
    digitalWrite(pinIN1_, LOW);
    digitalWrite(pinIN2_, LOW);
    ledcWrite(ledcCh_, 0);
    lastU_ = 0.0f;
}
