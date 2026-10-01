/**
 * encoder.h — đọc encoder Hall 2 kênh bằng ngoại vi PCNT (phần cứng) của ESP32.
 *
 * Vì sao PCNT: 2 bánh × 4281 xung/s = 8562 xung/s. Nếu dùng ngắt GPIO sẽ tốn
 * CPU và gây jitter cho task điều khiển. PCNT giải mã quadrature ×4 hoàn toàn
 * bằng phần cứng, CPU chỉ đọc thanh ghi 100 lần/giây.
 *
 * Xem docs/04_firmware-esp32.md §3.
 */
#pragma once
#include <Arduino.h>
#include "driver/pcnt.h"
#include "config.h"

class EncoderPCNT {
public:
    EncoderPCNT(uint8_t pinA, uint8_t pinB, pcnt_unit_t unit, bool invert);

    /** Cấu hình PCNT quadrature ×4 và bắt đầu đếm. */
    void begin();

    /**
     * Đọc số xung đã đếm từ lần gọi trước, sau đó XOÁ bộ đếm phần cứng.
     * Ở Ts = 10 ms và vận tốc tối đa, delta chỉ ≈ 43 xung  →  không bao giờ tràn.
     */
    int32_t readDelta();

    /** Tổng số xung tích luỹ (có dấu). */
    int64_t total() const { return total_; }

    /** Quãng đường tích luỹ của bánh (mét, có dấu). */
    float distanceM() const { return (float)total_ * DISTANCE_PER_COUNT_M; }

    void resetTotal() { total_ = 0; }

private:
    uint8_t      pinA_;
    uint8_t      pinB_;
    pcnt_unit_t  unit_;
    bool         invert_;
    int64_t      total_ = 0;
};
