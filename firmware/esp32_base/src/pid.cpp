#include "pid.h"

PID::PID() {}

void PID::configure(float kp, float ki, float kd,
                    float outMin, float outMax,
                    float iMin,   float iMax,
                    float dFilterTau) {
    kp_ = kp; ki_ = ki; kd_ = kd;
    outMin_ = outMin; outMax_ = outMax;
    iMin_ = iMin;     iMax_ = iMax;
    dTau_ = (dFilterTau > 1e-6f) ? dFilterTau : 0.02f;
    reset();
}

void PID::setGains(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void PID::reset() {
    integ_    = 0.0f;
    prevMeas_ = 0.0f;
    dFilt_    = 0.0f;
    first_    = true;
    lastOut_  = 0.0f;
    lastP_ = lastI_ = lastD_ = lastFF_ = 0.0f;
    saturated_ = false;
}

float PID::update(float sp, float meas, float dt, float ff) {
    if (dt <= 1e-6f) dt = CONTROL_DT_S;

    const float e = sp - meas;

    // ------------------------------------------------------------------
    //  P
    // ------------------------------------------------------------------
    const float P = kp_ * e;

    // ------------------------------------------------------------------
    //  D — đạo hàm theo ĐO LƯỜNG + lọc thông thấp bậc 1
    //      D = −Kd · d(y_f)/dt    (không bị "setpoint kick")
    // ------------------------------------------------------------------
    float D = 0.0f;
    if (first_) {
        // Bỏ qua mẫu đầu tiên: chưa có prevMeas để tính đạo hàm.
        dFilt_ = 0.0f;
        first_ = false;
    } else {
        const float dMeas = (meas - prevMeas_) / dt;          // đạo hàm thô
        const float a     = dt / (dTau_ + dt);                // hệ số LPF bậc 1
        dFilt_ += a * (dMeas - dFilt_);
        D = -kd_ * dFilt_;
    }
    prevMeas_ = meas;

    // ------------------------------------------------------------------
    //  I + anti-windup
    // ------------------------------------------------------------------
    const float I      = integ_;
    const float uUnsat = P + I + D + ff;
    const float uSat   = constrain(uUnsat, outMin_, outMax_);

#if   PID_ANTIWINDUP_MODE == 0
    // (0) Không anti-windup — CHỈ dùng để so sánh thí nghiệm. Nguy hiểm.
    integ_ += ki_ * dt * e;

#elif PID_ANTIWINDUP_MODE == 2
    // (2) Back-calculation: kéo I về khi ngõ ra bão hoà.
    integ_ += ki_ * dt * e + (uSat - uUnsat) * (dt / PID_BACKCALC_TT_S);

#elif PID_ANTIWINDUP_MODE == 3
    // (3) Integral separation: ngắt I khi sai số lớn.
    if (fabsf(e) < PID_INT_SEP_THRESH) {
        const bool pushingFurther =
            ((uUnsat > outMax_) && (e > 0.0f)) ||
            ((uUnsat < outMin_) && (e < 0.0f));
        if (!pushingFurther) integ_ += ki_ * dt * e;
    }

#else
    // (1) Clamping / conditional integration — KHUYẾN NGHỊ
    {
        const bool pushingFurther =
            ((uUnsat > outMax_) && (e > 0.0f)) ||
            ((uUnsat < outMin_) && (e < 0.0f));
        if (!pushingFurther) integ_ += ki_ * dt * e;
    }
#endif

    (void)uSat;      // một số chế độ anti-windup không dùng trực tiếp uSat

    // Lớp bảo vệ thứ hai: chặn biên khâu I
    integ_ = constrain(integ_, iMin_, iMax_);

    // ------------------------------------------------------------------
    //  Ngõ ra (dùng I TRƯỚC khi cập nhật — nhất quán với uUnsat ở trên)
    // ------------------------------------------------------------------
    const float u = constrain(P + I + D + ff, outMin_, outMax_);

    lastP_ = P; lastI_ = I; lastD_ = D; lastFF_ = ff;
    lastOut_   = u;
    saturated_ = (u != uUnsat);

    return u;
}
