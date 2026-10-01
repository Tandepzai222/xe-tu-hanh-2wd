#include "encoder.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#error "Firmware nay dung API PCNT cua Arduino-ESP32 core 2.x. Hay ghim platform = espressif32@6.9.0 trong platformio.ini, hoac viet lai encoder.cpp theo docs/04 muc 3.4."
#endif

// Bộ lọc nhiễu phần cứng: bỏ qua cạnh có độ rộng < 100 × 12.5 ns = 1.25 µs.
// Xung thật ở vận tốc tối đa có chu kỳ ≈ 234 µs → an toàn.
static const uint16_t PCNT_FILTER_VALUE = 100;

EncoderPCNT::EncoderPCNT(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit, bool invert)
    : pinA_(pinA), pinB_(pinB), unit_(unit), invert_(invert) {}

void EncoderPCNT::begin() {
    pcnt_config_t cfg = {};

    // ---------------------------------------------------------------
    // Kênh 0: A là xung đếm, B là tín hiệu điều khiển
    //   B = 0 → đếm ngược,  B = 1 → đếm xuôi
    //   cạnh lên A → +1,    cạnh xuống A → −1
    // ---------------------------------------------------------------
    cfg.pulse_gpio_num = (int16_t)pinA_;
    cfg.ctrl_gpio_num  = (int16_t)pinB_;
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_REVERSE;
    cfg.pos_mode       = PCNT_COUNT_INC;
    cfg.neg_mode       = PCNT_COUNT_DEC;
    cfg.counter_h_lim  =  32767;
    cfg.counter_l_lim  = -32768;
    cfg.unit           = unit_;
    cfg.channel        = PCNT_CHANNEL_0;
    pcnt_unit_config(&cfg);

    // ---------------------------------------------------------------
    // Kênh 1: B là xung đếm, A là tín hiệu điều khiển (đảo chiều logic)
    //   A = 0 → đếm xuôi,   A = 1 → đếm ngược
    //   cạnh lên B → −1,    cạnh xuống B → +1
    // ---------------------------------------------------------------
    cfg.pulse_gpio_num = (int16_t)pinB_;
    cfg.ctrl_gpio_num  = (int16_t)pinA_;
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_REVERSE;
    cfg.pos_mode       = PCNT_COUNT_DEC;
    cfg.neg_mode       = PCNT_COUNT_INC;
    cfg.channel        = PCNT_CHANNEL_1;
    pcnt_unit_config(&cfg);

    // Bộ lọc + khởi động
    pcnt_counter_pause(unit_);
    pcnt_counter_clear(unit_);
    pcnt_set_filter_value(unit_, PCNT_FILTER_VALUE);
    pcnt_filter_enable(unit_);
    pcnt_counter_resume(unit_);
}

int32_t EncoderPCNT::readDelta() {
    int16_t raw = 0;
    pcnt_get_counter_value(unit_, &raw);
    pcnt_counter_clear(unit_);          // xoá mỗi chu kỳ → không bao giờ tràn

    int32_t d = (int32_t)raw;
    if (invert_) d = -d;
    total_ += d;
    return d;
}
