/**
 * safety.h — giám sát an toàn: pin yếu, kẹt bánh, mất lệnh, e-stop.
 * Xem docs/04 §5.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ---- Bit trạng thái (gửi trong telemetry, trường "st") ----
#define ST_STALL_L       0x0001
#define ST_STALL_R       0x0002
#define ST_BATT_LOW      0x0004
#define ST_ESTOP         0x0008
#define ST_IMU_FAIL      0x0010
#define ST_CMD_TIMEOUT   0x0020
#define ST_WIFI_DOWN     0x0040
#define ST_WARN          0x0080

class SafetyMonitor {
public:
    void begin();

    /**
     * @param vbat         điện áp pin đo được (V)
     * @param dutyL/dutyR  |PWM chuẩn hoá| thực tế 0..1
     * @param vL/vR        vận tốc đo được (m/s)
     * @param remoteActive robot đang được điều khiển từ xa (bật watchdog lệnh)
     * @param cmdFresh     có lệnh mới trong khoảng CMD_TIMEOUT_MS
     * @param dt           chu kỳ (s)
     */
    void update(float vbat, float dutyL, float dutyR,
                float vL, float vR,
                bool remoteActive, bool cmdFresh, float dt);

    uint16_t bits() const { return bits_; }
    void     setBit(uint16_t b)   { bits_ |= b; }
    void     clearBit(uint16_t b) { bits_ &= (uint16_t)~b; }

    /** Xoá các lỗi có thể xoá được (giữ lại ESTOP nếu chưa `reset`). */
    void clearRecoverable();

    /** Lỗi buộc dừng robot ngay. */
    bool hasFatal() const {
        return (bits_ & (ST_STALL_L | ST_STALL_R | ST_ESTOP | ST_BATT_LOW)) != 0;
    }

    float vbatFilt() const { return vbatFilt_; }

private:
    uint16_t bits_       = 0;
    float    vbatFilt_   = 0.0f;
    bool     vbatFirst_  = true;
    float    stallL_     = 0.0f;
    float    stallR_     = 0.0f;
};
