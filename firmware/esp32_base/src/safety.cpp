#include "safety.h"

void SafetyMonitor::begin() {
    bits_      = 0;
    vbatFilt_  = 0.0f;
    vbatFirst_ = true;
    stallL_    = 0.0f;
    stallR_    = 0.0f;
}

void SafetyMonitor::clearRecoverable() {
    // ESTOP chỉ được xoá bằng lệnh `reset` tường minh, không xoá ở đây.
    bits_ &= ST_ESTOP;
}

void SafetyMonitor::update(float vbat, float dutyL, float dutyR,
                           float vL, float vR,
                           bool remoteActive, bool cmdFresh, float dt) {
    // ---------------- Lọc điện áp pin ----------------
    if (vbatFirst_) { vbatFilt_ = vbat; vbatFirst_ = false; }
    else            { vbatFilt_ += 0.05f * (vbat - vbatFilt_); }

    // ---------------- Pin yếu / nguy hiểm (có tự hồi phục) ----------------
    if (vbatFilt_ > 1.0f && vbatFilt_ < BATT_CRIT_V) {
        bits_ |= ST_BATT_LOW;
        bits_ |= ST_WARN;
    } else if (vbatFilt_ > 1.0f && vbatFilt_ < BATT_WARN_V) {
        bits_ |= ST_WARN;
        bits_ &= (uint16_t)~ST_BATT_LOW;
    } else if (vbatFilt_ > 1.0f) {
        // điện áp đã hồi phục → tự xoá cờ cảnh báo/thấp
        bits_ &= (uint16_t)~(ST_BATT_LOW | ST_WARN);
    }

    // ---------------- Phát hiện kẹt bánh ----------------
    // Điều kiện: PWM lớn NHƯNG bánh không quay, kéo dài STALL_TIME_MS.
    const bool stallCondL = (dutyL > STALL_PWM_THRESH) && (fabsf(vL) < STALL_VEL_THRESH);
    const bool stallCondR = (dutyR > STALL_PWM_THRESH) && (fabsf(vR) < STALL_VEL_THRESH);

    stallL_ = stallCondL ? (stallL_ + dt) : 0.0f;
    stallR_ = stallCondR ? (stallR_ + dt) : 0.0f;

    if (stallL_ * 1000.0f > STALL_TIME_MS) bits_ |= ST_STALL_L;
    if (stallR_ * 1000.0f > STALL_TIME_MS) bits_ |= ST_STALL_R;

    // ---------------- Watchdog lệnh từ xa ----------------
#if CMD_TIMEOUT_ENABLE
    if (remoteActive && !cmdFresh) bits_ |= ST_CMD_TIMEOUT;
    else                           bits_ &= (uint16_t)~ST_CMD_TIMEOUT;
#endif
}
