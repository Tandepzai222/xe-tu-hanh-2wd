#include "robot.h"
#include <math.h>
#include <string.h>

// ============================================================================
//  Biến chia sẻ giữa controlTask (core 1, prio 10) và loopTask (core 1, prio 1)
// ============================================================================
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Bộ đếm thời gian cho chế độ hiệu chuẩn vùng chết (file-static: chỉ có 1 Robot)
static uint32_t g_dzLastIncMs = 0;

// ============================================================================
//  Tiện ích nội bộ
// ============================================================================
const char* modeName(Mode m) {
    switch (m) {
        case Mode::IDLE:    return "IDLE";
        case Mode::DUTY:    return "DUTY";
        case Mode::WHEEL:   return "WHEEL";
        case Mode::BODY:    return "BODY";
        case Mode::HEADING: return "HEADING";
        case Mode::GOTO:    return "GOTO";
        case Mode::SPIN:    return "SPIN";
        case Mode::STEP:    return "STEP";
        case Mode::RELAY:   return "RELAY";
        case Mode::CAL_DZ:  return "CAL_DZ";
        default:            return "?";
    }
}

float Robot::wrapToPi(float a) {
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

static inline float slewStep(float current, float target, float maxDelta) {
    const float d = target - current;
    if (d >  maxDelta) return current + maxDelta;
    if (d < -maxDelta) return current - maxDelta;
    return target;
}

/**
 * Giới hạn động học: nếu (v, ω) yêu cầu bánh vượt V_MAX thì scale CẢ HAI
 * theo cùng một hệ số → giữ nguyên tỉ lệ v/ω → giữ nguyên quỹ đạo.
 * Xem docs/02 §2.4c.
 */
static inline void clampKinematics(float& v, float& w) {
    v = constrain(v, -V_MAX_MPS, V_MAX_MPS);
    w = constrain(w, -W_MAX_RAD_S, W_MAX_RAD_S);

    const float half = WHEEL_TRACK_M * 0.5f;
    float vl = v - w * half;
    float vr = v + w * half;
    const float m = fmaxf(fabsf(vl), fabsf(vr));

    if (m > V_MAX_MPS && m > 1e-6f) {
        const float s = V_MAX_MPS / m;
        v *= s;
        w *= s;
    }
}

// ============================================================================
//  Khởi tạo
// ============================================================================
Robot::Robot()
    : encL_(PIN_ENC_L_A, PIN_ENC_L_B, PCNT_UNIT_0, ENC_L_INVERT),
      encR_(PIN_ENC_R_A, PIN_ENC_R_B, PCNT_UNIT_1, ENC_R_INVERT),
      motL_(PIN_MOT_L_PWM, PIN_MOT_L_IN1, PIN_MOT_L_IN2, LEDC_CH_L, MOTOR_L_INVERT),
      motR_(PIN_MOT_R_PWM, PIN_MOT_R_IN1, PIN_MOT_R_IN2, LEDC_CH_R, MOTOR_R_INVERT) {}

void Robot::begin() {
    // ------------------------------------------------------------------
    // ⚠️ QUAN TRỌNG: tắt driver TB6612 NGAY LẬP TỨC, trước khi cấu hình bất
    //    cứ thứ gì. Nhờ vậy các xung glitch lúc boot trên GPIO 16/17/4/25/26/27 không
    //    làm robot giật. Chỉ bật lại khi mọi thứ đã sẵn sàng.
    // ------------------------------------------------------------------
    pinMode(PIN_TB_STBY, OUTPUT);
    digitalWrite(PIN_TB_STBY, LOW);

    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);

    encL_.begin();
    encR_.begin();
    motL_.begin();
    motR_.begin();

    pidL_.configure(par_.kpL, par_.kiL, par_.kdL,
                    PID_OUT_MIN, PID_OUT_MAX, PID_I_MIN, PID_I_MAX,
                    PID_D_FILTER_TAU_S);
    pidR_.configure(par_.kpR, par_.kiR, par_.kdR,
                    PID_OUT_MIN, PID_OUT_MAX, PID_I_MIN, PID_I_MAX,
                    PID_D_FILTER_TAU_S);

    safety_.begin();

    // Hiệu chuẩn bias gyro được gọi riêng từ main.cpp (in cảnh báo rõ ràng)
    if (!imu_.begin()) {
        setImuFail();
        Serial.println("[ROBOT] CANH BAO: IMU khong hoat dong. "
                       "Cac che do HEADING/GOTO se khong dung duoc.");
    }

    readBattery();
    resetControllers();
    lastImuUs_  = micros();
    lastBattMs_ = millis();

    // Bật driver
    digitalWrite(PIN_TB_STBY, HIGH);
    motL_.setEnabled(true);
    motR_.setEnabled(true);

    Serial.printf("[ROBOT] San sang. D=%.1f mm  L=%.1f mm  N=%.1f xung/vong  d_pc=%.5f mm\n",
                  WHEEL_DIAMETER_M * 1000.0f, WHEEL_TRACK_M * 1000.0f,
                  COUNTS_PER_WHEEL_REV, DISTANCE_PER_COUNT_M * 1000.0f);
    Serial.printf("[ROBOT] v_max=%.4f m/s   w_max=%.3f rad/s   v_safe=%.2f m/s\n",
                  V_MAX_MPS, W_MAX_RAD_S, V_SAFE_MPS);
}

void Robot::resetControllers() {
    pidL_.reset();
    pidR_.reset();
    vspL_ = 0.0f;
    vspR_ = 0.0f;
    velFirst_ = true;
    vL_ = vR_ = 0.0f;
}

// ============================================================================
//  API lệnh
// ============================================================================
void Robot::cmdSetMode(Mode m, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = m;
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);

    if (m == Mode::IDLE) resetControllers();
    if (m == Mode::HEADING) {
        // neo hướng tham chiếu nếu chưa có
    }
}

void Robot::cmdOpenLoop(float ul, float ur, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::DUTY;
    cmd_.ul = constrain(ul, -1.0f, 1.0f);
    cmd_.ur = constrain(ur, -1.0f, 1.0f);
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdWheel(float vl, float vr, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::WHEEL;
    cmd_.vl = constrain(vl, -V_MAX_MPS, V_MAX_MPS);
    cmd_.vr = constrain(vr, -V_MAX_MPS, V_MAX_MPS);
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdBody(float v, float w, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::BODY;
    cmd_.v = v;
    cmd_.w = w;
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdHeading(float v, float thRefRad, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::HEADING;
    cmd_.v = v;
    cmd_.thRef = thRefRad;
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdGoto(float xg, float yg, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::GOTO;
    cmd_.gx = xg;
    cmd_.gy = yg;
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdSpin(float w, bool remote) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::SPIN;
    cmd_.w = constrain(w, -W_MAX_RAD_S, W_MAX_RAD_S);
    if (remote) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

void Robot::cmdStepTest(float pwm, uint32_t ms) {
    // `pwm` là giá trị duty thô 0..PWM_MAX (giống lệnh `mode duty`)
    const uint32_t duty = (uint32_t)constrain(pwm, 0.0f, (float)PWM_MAX);
    portENTER_CRITICAL(&g_mux);
    cmd_.mode      = Mode::STEP;
    cmd_.stepPwm   = duty;
    cmd_.stepEndMs = millis() + ms;
    portEXIT_CRITICAL(&g_mux);
    resetControllers();
    Serial.printf("[STEP] duty=%lu/%d trong %lu ms. Bat dau ghi CSV...\n",
                  (unsigned long)duty, PWM_MAX, (unsigned long)ms);
}

void Robot::cmdRelayTune(uint8_t wheel, float vsp) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::RELAY;
    portEXIT_CRITICAL(&g_mux);

    relay_.active    = true;
    relay_.wheel     = (wheel == 0) ? 0 : 1;
    relay_.vsp       = vsp;
    relay_.sign      = 1;
    relay_.tSwitch   = (float)millis() * 0.001f;
    relay_.halfSum   = 0.0f;
    relay_.halfCnt   = 0;
    relay_.switchCnt = 0;
    relay_.vMin      = 1e9f;
    relay_.vMax      = -1e9f;
    relay_.done      = false;
    resetControllers();

    Serial.printf("[RELAY] Auto-tune banh %s, setpoint %.3f m/s. Cho 10-20 giay...\n",
                  relay_.wheel == 0 ? "TRAI" : "PHAI", vsp);
}

void Robot::cmdCalDeadzone(uint8_t wheel) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::CAL_DZ;
    portEXIT_CRITICAL(&g_mux);

    dz_.active = true;
    dz_.wheel  = (wheel == 0) ? 0 : 1;
    dz_.duty   = 0.0f;
    dz_.done   = false;
    dz_.result = 0.0f;
    g_dzLastIncMs = millis();
    resetControllers();

    Serial.printf("[CAL] Tim vung chet banh %s. PWM se tang 1 don vi moi 100 ms.\n",
                  dz_.wheel == 0 ? "TRAI" : "PHAI");
    Serial.println("[CAL] (Banh phai quay tu do tren gia!)");
}

void Robot::cmdStop(bool emergency) {
    portENTER_CRITICAL(&g_mux);
    cmd_.mode = Mode::IDLE;
    cmd_.vl = cmd_.vr = 0.0f;
    cmd_.v = cmd_.w = 0.0f;
    cmd_.ul = cmd_.ur = 0.0f;
    relay_.active = false;
    dz_.active    = false;
    portEXIT_CRITICAL(&g_mux);

    if (emergency) {
        safety_.setBit(ST_ESTOP);
        motL_.brake();
        motR_.brake();
        digitalWrite(PIN_TB_STBY, HIGH);
        Serial.println("[ESTOP] Da dung khan cap. Go lenh `clear` de xoa.");
    } else {
        resetControllers();
        Serial.println("[STOP] Da dung.");
    }
}

void Robot::cmdClearErrors() {
    safety_.clearRecoverable();
    safety_.clearBit(ST_ESTOP);
    digitalWrite(PIN_TB_STBY, HIGH);
    motL_.setEnabled(true);
    motR_.setEnabled(true);
    resetControllers();
    Serial.println("[CLEAR] Da xoa co loi.");
}

void Robot::cmdResetOdometry() {
    portENTER_CRITICAL(&g_mux);
    pose_ = Pose2D();
    portEXIT_CRITICAL(&g_mux);
    encL_.resetTotal();
    encR_.resetTotal();
    Serial.println("[ODOM] Da reset ve (0, 0, 0).");
}

void Robot::cmdResetYaw() {
    imu_.resetYaw(0.0f);
    portENTER_CRITICAL(&g_mux);
    cmd_.thRef = 0.0f;
    portEXIT_CRITICAL(&g_mux);
    Serial.println("[IMU] Da reset yaw = 0.");
}

bool Robot::cmdCalImu(uint16_t samples) {
    const bool ok = imu_.calibrateGyro(samples);
    if (ok) safety_.clearBit(ST_IMU_FAIL);
    else    safety_.setBit(ST_IMU_FAIL);
    return ok;
}

bool Robot::cmdSetGain(char wheel, const char* which, float value) {
    const bool isKp = (strcmp(which, "kp") == 0);
    const bool isKi = (strcmp(which, "ki") == 0);
    const bool isKd = (strcmp(which, "kd") == 0);
    if (!isKp && !isKi && !isKd) return false;

    if (wheel == 'l') {
        if (isKp) par_.kpL = value;
        if (isKi) par_.kiL = value;
        if (isKd) par_.kdL = value;
        pidL_.setGains(par_.kpL, par_.kiL, par_.kdL);
        Serial.printf("[PID] Banh TRAI: kp=%.4f ki=%.4f kd=%.4f\n", par_.kpL, par_.kiL, par_.kdL);
        return true;
    }
    if (wheel == 'r') {
        if (isKp) par_.kpR = value;
        if (isKi) par_.kiR = value;
        if (isKd) par_.kdR = value;
        pidR_.setGains(par_.kpR, par_.kiR, par_.kdR);
        Serial.printf("[PID] Banh PHAI: kp=%.4f ki=%.4f kd=%.4f\n", par_.kpR, par_.kiR, par_.kdR);
        return true;
    }
    if (wheel == 'h') {
        if (isKp) par_.headingKp = value;
        if (isKd) par_.headingKd = value;
        Serial.printf("[PID] Huong: kp=%.4f kd=%.4f\n", par_.headingKp, par_.headingKd);
        return true;
    }
    return false;
}

bool Robot::cmdSetParam(const char* key, float value) {
    if      (strcmp(key, "motor_k")   == 0) par_.motorK   = value;
    else if (strcmp(key, "dz")        == 0) { par_.dzL = value; par_.dzR = value; }
    else if (strcmp(key, "dzl")       == 0) par_.dzL      = value;
    else if (strcmp(key, "dzr")       == 0) par_.dzR      = value;
    else if (strcmp(key, "ff_gain")   == 0) par_.ffGain   = value;
    else if (strcmp(key, "acc_max")   == 0) par_.accMax   = value;
    else if (strcmp(key, "alpha_max") == 0) par_.alphaMax = value;
    else if (strcmp(key, "vel_tau")   == 0) par_.velFiltTau = value;
    else if (strcmp(key, "heading_kp")== 0) par_.headingKp = value;
    else if (strcmp(key, "heading_kd")== 0) par_.headingKd = value;
    else return false;

    Serial.printf("[CFG] %s = %.4f\n", key, value);
    return true;
}

void Robot::setImuFail() {
    safety_.setBit(ST_IMU_FAIL);
}

void Robot::setStatusBit(uint16_t bit, bool value) {
    if (value) safety_.setBit(bit);
    else       safety_.clearBit(bit);
}

void Robot::setRemoteAlive(bool alive) {
    portENTER_CRITICAL(&g_mux);
    if (alive) { cmd_.remoteSeen = true; cmd_.lastRemoteMs = millis(); }
    portEXIT_CRITICAL(&g_mux);
}

// ============================================================================
//  Vòng điều khiển — gọi từ controlTask mỗi Ts
// ============================================================================
void Robot::step(float dt) {
    if (dt < DT_MIN_S) dt = DT_MIN_S;
    if (dt > DT_MAX_S) dt = DT_MAX_S;

    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();

    // ---- đo jitter + tần số thực ----
    {
        const float jit = fabsf(dt - CONTROL_DT_S) * 1e6f;
        jitter_ = (uint32_t)jit;
        const float instHz = 1.0f / dt;
        hz_ += 0.05f * (instHz - hz_);
    }

    // ---- 1. IMU (tần số riêng 200 Hz) ----
    if ((uint32_t)(nowUs - lastImuUs_) >= (uint32_t)IMU_PERIOD_MS * 1000u) {
        const float idt = (float)(nowUs - lastImuUs_) * 1e-6f;
        lastImuUs_ = nowUs;
        imu_.poll(idt);
    }

    // ---- 2. Encoder ----
    const int32_t dL = encL_.readDelta();
    const int32_t dR = encR_.readDelta();

    // ---- 3. Vận tốc (thô + lọc LPF bậc 1) ----
    vLraw_ = (float)dL * DISTANCE_PER_COUNT_M / dt;
    vRraw_ = (float)dR * DISTANCE_PER_COUNT_M / dt;
    if (velFirst_) {
        vL_ = vLraw_; vR_ = vRraw_; velFirst_ = false;
    } else {
        const float a = par_.velFiltTau / (par_.velFiltTau + dt);   // τ/(τ+Ts)
        vL_ = a * vL_ + (1.0f - a) * vLraw_;
        vR_ = a * vR_ + (1.0f - a) * vRraw_;
    }

    // ---- 4. Odometry ----
    integrateOdometry(dL, dR);

    // ---- 5. Điện áp pin (10 Hz) ----
    if ((uint32_t)(nowMs - lastBattMs_) >= 100u) {
        lastBattMs_ = nowMs;
        readBattery();
    }

    // ---- 6. Lấy lệnh ----
    Cmd c;
    portENTER_CRITICAL(&g_mux);
    c = cmd_;
    portEXIT_CRITICAL(&g_mux);

    // ---- 7. Tính setpoint ----
    float vspL = 0.0f, vspR = 0.0f, uDirL = 0.0f, uDirR = 0.0f;
    bool  useDirect = false, allowDrive = true;
    computeWheelSetpoints(dt, vspL, vspR, uDirL, uDirR, useDirect, allowDrive);

    if (c.mode == Mode::RELAY)  handleRelay(dt, uDirL, uDirR, useDirect, allowDrive);
    if (c.mode == Mode::CAL_DZ) handleDeadzoneCal(uDirL, uDirR, useDirect, allowDrive);

    // ---- 8. Ramp vận tốc (slew limiting) ----
    if (!useDirect) {
#if SLEW_ENABLE
        const float dvMax = par_.accMax * dt;
        vspL_ = slewStep(vspL_, vspL, dvMax);
        vspR_ = slewStep(vspR_, vspR, dvMax);
#else
        vspL_ = vspL; vspR_ = vspR;
#endif
    } else {
        vspL_ = vspL;
        vspR_ = vspR;
    }

    // ---- 9. PID + feedforward ----
    float uL = 0.0f, uR = 0.0f;
    if (useDirect) {
        uL = uDirL;
        uR = uDirR;
    } else {
#if FF_ENABLE
        const float ffL = computeFeedforward(vspL_, par_.dzL) * par_.ffGain;
        const float ffR = computeFeedforward(vspR_, par_.dzR) * par_.ffGain;
#else
        const float ffL = 0.0f, ffR = 0.0f;
#endif
        uL = pidL_.update(vspL_, vL_, dt, ffL);
        uR = pidR_.update(vspR_, vR_, dt, ffR);
    }

    // ---- 10. An toàn ----
    const bool remoteActive =
        (c.mode == Mode::BODY || c.mode == Mode::WHEEL) && c.remoteSeen;
    const bool cmdFresh =
        c.remoteSeen && ((uint32_t)(nowMs - c.lastRemoteMs) < (uint32_t)CMD_TIMEOUT_MS);

    safety_.update(vbat_, fabsf(uL), fabsf(uR), vL_, vR_, remoteActive, cmdFresh, dt);

    if (safety_.bits() & ST_ESTOP) {
        motL_.brake();
        motR_.brake();
        digitalWrite(PIN_TB_STBY, HIGH);
        uL = uR = 0.0f;
        allowDrive = false;
    } else if (safety_.hasFatal()) {
        // Kẹt bánh hoặc pin nguy hiểm → cắt ngõ ra ngay (không ramp)
        uL = uR = 0.0f;
        allowDrive = false;
    }
    // Trường hợp ST_CMD_TIMEOUT (mất lệnh từ xa): computeWheelSetpoints() đã ép
    // setpoint về 0 nên PID ở trên đã tự sinh ngõ ra HÃM có kiểm soát, và
    // slew limiter giới hạn tốc độ giảm → robot dừng êm, không bị giật.
    // Không cần xử lý thêm ở đây.

    if (!allowDrive) { uL = 0.0f; uR = 0.0f; }

    // ---- 11. Xuất PWM ----
    motL_.setDuty(uL);
    motR_.setDuty(uR);

    // ---- 12. Cập nhật telemetry ----
    portENTER_CRITICAL(&g_mux);
    tel_.t_ms     = nowMs;
    tel_.x        = pose_.x;
    tel_.y        = pose_.y;
    tel_.th       = pose_.th;
    tel_.v        = 0.5f * (vL_ + vR_);
    tel_.w        = (vR_ - vL_) / WHEEL_TRACK_M;
    tel_.vl       = vL_;
    tel_.vr       = vR_;
    tel_.vspL     = vspL_;
    tel_.vspR     = vspR_;
    tel_.cl       = encL_.total();
    tel_.cr       = encR_.total();
    tel_.ax       = imu_.data().ax;
    tel_.ay       = imu_.data().ay;
    tel_.az       = imu_.data().az;
    tel_.gx       = imu_.data().gx;
    tel_.gy       = imu_.data().gy;
    tel_.gz       = imu_.data().gz;
    tel_.roll     = imu_.data().roll;
    tel_.pitch    = imu_.data().pitch;
    tel_.yaw      = imu_.data().yaw;
    tel_.tempC    = imu_.data().tempC;
    tel_.pwm_l    = (int16_t)(motL_.lastDuty() * (float)PWM_MAX);
    tel_.pwm_r    = (int16_t)(motR_.lastDuty() * (float)PWM_MAX);
    tel_.vbat     = vbat_;
    tel_.status   = safety_.bits();
    tel_.hz       = hz_;
    tel_.jitter_us= jitter_;
    tel_.seq      = ++telSeq_;
    tel_.mode     = c.mode;
    tel_.imuOk    = imu_.ok() && imu_.calibrated();
    tel_.whoAmI   = imu_.whoAmI();
    tel_.imuBiasZ = imu_.biasZ();
    tel_.pidPL    = pidL_.lastP();
    tel_.pidIL    = pidL_.lastI();
    tel_.pidDL    = pidL_.lastD();
    tel_.pidFFL   = pidL_.lastFF();
    tel_.pidPR    = pidR_.lastP();
    tel_.pidIR    = pidR_.lastI();
    tel_.pidDR    = pidR_.lastD();
    tel_.pidFFR   = pidR_.lastFF();
    portEXIT_CRITICAL(&g_mux);
}

// ============================================================================
//  Tính setpoint bánh theo chế độ
// ============================================================================
void Robot::computeWheelSetpoints(float dt, float& vspL, float& vspR,
                                  float& ulDirect, float& urDirect,
                                  bool& useDirect, bool& allowDrive) {
    (void)dt;

    Cmd c;
    portENTER_CRITICAL(&g_mux);
    c = cmd_;
    portEXIT_CRITICAL(&g_mux);

    vspL = vspR = 0.0f;
    ulDirect = urDirect = 0.0f;
    useDirect = false;
    allowDrive = true;

    // ---- Lỗi nghiêm trọng → dừng ----
    if (safety_.bits() & (ST_ESTOP | ST_STALL_L | ST_STALL_R | ST_BATT_LOW)) {
        useDirect  = true;
        allowDrive = false;
        return;
    }

    switch (c.mode) {
        // ------------------------------------------------------------
        case Mode::IDLE:
            useDirect  = true;
            allowDrive = false;
            break;

        // ------------------------------------------------------------
        case Mode::DUTY:
            useDirect = true;
            ulDirect  = c.ul;
            urDirect  = c.ur;
            break;

        // ------------------------------------------------------------
        case Mode::WHEEL:
        case Mode::BODY:
        case Mode::SPIN:
        case Mode::HEADING:
        case Mode::GOTO: {
            // Watchdog chỉ áp dụng cho chế độ điều khiển từ xa (WHEEL/BODY)
            if ((c.mode == Mode::WHEEL || c.mode == Mode::BODY) && c.remoteSeen) {
                const bool fresh = ((uint32_t)(millis() - c.lastRemoteMs) < (uint32_t)CMD_TIMEOUT_MS);
                if (!fresh) {
                    vspL = vspR = 0.0f;      // ramp về 0 nhờ slew + PID
                    break;
                }
            }

            float v = 0.0f, w = 0.0f;

            if (c.mode == Mode::WHEEL) {
                vspL = c.vl;
                vspR = c.vr;
                break;                        // đã là vận tốc bánh
            } else if (c.mode == Mode::BODY) {
                v = c.v;
                w = c.w;
            } else if (c.mode == Mode::SPIN) {
                v = 0.0f;
                w = c.w;
            } else if (c.mode == Mode::HEADING) {
                const float eth = wrapToPi(c.thRef - imu_.data().yaw);
                w = par_.headingKp * eth - par_.headingKd * imu_.data().gz;
                w = constrain(w, -HEADING_W_MAX, HEADING_W_MAX);
                v = c.v;
            } else {                          // GOTO
                const float dx = c.gx - pose_.x;
                const float dy = c.gy - pose_.y;
                const float ed = sqrtf(dx * dx + dy * dy);

                if (ed < GOTO_TOL_M) {
                    portENTER_CRITICAL(&g_mux);
                    cmd_.mode = Mode::IDLE;
                    portEXIT_CRITICAL(&g_mux);
                    v = 0.0f; w = 0.0f;
                    useDirect  = true;
                    allowDrive = false;
                    return;
                }

                const float phi  = atan2f(dy, dx);
                const float ephi = wrapToPi(phi - pose_.th);

                w = par_.headingKp * ephi - par_.headingKd * imu_.data().gz;
                w = constrain(w, -HEADING_W_MAX, HEADING_W_MAX);

                const float cph = cosf(ephi);
                v = (cph > 0.05f) ? (GOTO_KDIST * ed * cph) : 0.0f;
                v = constrain(v, 0.0f, GOTO_V_MAX);
            }

            // Giới hạn động học: giữ nguyên tỉ lệ v/ω (docs/02 §2.4c)
            clampKinematics(v, w);
            vspL = v - w * WHEEL_TRACK_M * 0.5f;
            vspR = v + w * WHEEL_TRACK_M * 0.5f;
            break;
        }

        // ------------------------------------------------------------
        case Mode::STEP: {
            useDirect = true;
            if ((uint32_t)millis() < c.stepEndMs) {
                const float u = (float)c.stepPwm / (float)PWM_MAX;
                ulDirect = u;
                urDirect = u;
                vspL = vspR = 0.0f;
            } else {
                ulDirect = urDirect = 0.0f;
                allowDrive = false;
                portENTER_CRITICAL(&g_mux);
                cmd_.mode = Mode::IDLE;
                portEXIT_CRITICAL(&g_mux);
                Serial.println("[STEP] Ket thuc.");
            }
            break;
        }

        // ------------------------------------------------------------
        case Mode::RELAY:
        case Mode::CAL_DZ:
            useDirect  = true;
            allowDrive = false;      // sẽ được bật bởi handler tương ứng
            break;

        default:
            useDirect  = true;
            allowDrive = false;
            break;
    }
}

// ============================================================================
//  Relay auto-tuning (Åström–Hägglund) — docs/03 §6.2
// ============================================================================
void Robot::handleRelay(float dt, float& ul, float& ur, bool& useDirect, bool& allowDrive) {
    (void)dt;
    RelayState& R = relay_;
    useDirect = true;

    if (!R.active) {
        allowDrive = false;
        ul = ur = 0.0f;
        return;
    }
    allowDrive = true;

    const float RELAY_D = 0.20f;    // biên độ relay (20 % PWM)
    const float RELAY_H = 0.010f;   // hysteresis (m/s)

    const float v    = (R.wheel == 0) ? vL_ : vR_;
    const float tNow = (float)millis() * 0.001f;

    // đo cực trị để suy ra biên độ dao động
    if (v < R.vMin) R.vMin = v;
    if (v > R.vMax) R.vMax = v;

    // logic relay có trễ
    const float e = R.vsp - v;
    int8_t newSign = R.sign;
    if (R.sign > 0 && e < -RELAY_H)      newSign = -1;
    else if (R.sign < 0 && e > +RELAY_H) newSign = +1;

    if (newSign != R.sign) {
        const float half = tNow - R.tSwitch;
        R.tSwitch = tNow;
        R.sign    = newSign;
        R.switchCnt++;

        if (R.switchCnt == 3) {
            // bắt đầu đo sau khi hết quá độ ban đầu
            R.vMin = 1e9f; R.vMax = -1e9f;
            R.halfSum = 0.0f; R.halfCnt = 0;
        } else if (R.switchCnt > 3) {
            R.halfSum += half;
            R.halfCnt++;
        }

        if (R.halfCnt >= 6) {
            const float Tu  = (R.halfSum / (float)R.halfCnt) * 2.0f;
            const float amp = 0.5f * (R.vMax - R.vMin);

            if (amp > 1e-3f && Tu > 0.02f) {
                const float Ku = 4.0f * RELAY_D / ((float)M_PI * amp);

                const float znKp = 0.60f * Ku,  znTi = 0.5f  * Tu;
                const float tlKp = Ku / 2.2f,   tlTi = 2.2f  * Tu;
                const float noKp = 0.20f * Ku,  noTi = 0.5f  * Tu;

                Serial.println();
                Serial.println("========== KET QUA RELAY AUTO-TUNE ==========");
                Serial.printf("Bien do dao dong a  = %.4f m/s\n", amp);
                Serial.printf("Chu ky dao dong Tu  = %.3f s\n", Tu);
                Serial.printf("Diem cuc dai Ku     = %.4f   (d = %.2f, h = %.3f)\n",
                              Ku, RELAY_D, RELAY_H);
                Serial.println();
                Serial.printf("  Ziegler-Nichols : Kp=%.3f  Ki=%.3f   (overshoot ~25%%, KHONG khuyen nghi)\n",
                              znKp, znKp / znTi);
                Serial.printf("  Tyreus-Luyben   : Kp=%.3f  Ki=%.3f   <-- KHUYEN NGHI\n",
                              tlKp, tlKp / tlTi);
                Serial.printf("  No-overshoot    : Kp=%.3f  Ki=%.3f\n",
                              noKp, noKp / noTi);
                Serial.println();
                Serial.printf("Nap gain bang lenh:  pid %c kp <Kp>   va   pid %c ki <Ki>\n",
                              R.wheel == 0 ? 'l' : 'r', R.wheel == 0 ? 'l' : 'r');
                Serial.println("=============================================");
            } else {
                Serial.println("[RELAY] Khong do duoc dao dong (bien do qua nho). "
                               "Tang bien do relay hoac kiem tra banh co quay tu do khong.");
            }
            R.active = false;
            R.done   = true;
            portENTER_CRITICAL(&g_mux);
            cmd_.mode = Mode::IDLE;
            portEXIT_CRITICAL(&g_mux);
            return;
        }
    }

    const float u = (R.sign > 0) ? +RELAY_D : -RELAY_D;
    if (R.wheel == 0) { ul = u; ur = 0.0f; }
    else              { ul = 0.0f; ur = u; }
}

// ============================================================================
//  Hiệu chuẩn vùng chết PWM — docs/02 §7.3
// ============================================================================
void Robot::handleDeadzoneCal(float& ul, float& ur, bool& useDirect, bool& allowDrive) {
    DzCal& D = dz_;
    useDirect = true;

    if (!D.active) {
        allowDrive = false;
        ul = ur = 0.0f;
        return;
    }
    allowDrive = true;

    const uint32_t now = millis();
    if ((uint32_t)(now - g_dzLastIncMs) >= 100u) {
        g_dzLastIncMs = now;
        D.duty += 1.0f;
    }

    const float v = (D.wheel == 0) ? vLraw_ : vRraw_;

    if (fabsf(v) > 0.005f && D.duty > 20.0f) {
        D.result = D.duty / (float)PWM_MAX;
        if (D.wheel == 0) par_.dzL = D.result; else par_.dzR = D.result;
        D.done = true;
        D.active = false;
        Serial.println();
        Serial.printf("[CAL] => VUNG CHET banh %s: PWM = %.0f / %d  (%.1f%%)  -> DZ = %.4f\n",
                      D.wheel == 0 ? "TRAI" : "PHAI", D.duty, PWM_MAX,
                      100.0f * D.result, D.result);
        Serial.println("[CAL] Dung lenh `cfg dzl <x>` hoac `cfg dzr <x>` de luu gia tri nay.");
        portENTER_CRITICAL(&g_mux);
        cmd_.mode = Mode::IDLE;
        portEXIT_CRITICAL(&g_mux);
        ul = ur = 0.0f;
        allowDrive = false;
        return;
    }

    if (D.duty > 500.0f) {
        Serial.println("[CAL] CANH BAO: PWM > 500 ma banh khong quay. "
                       "Kiem tra co khi / nguon / day motor.");
        D.active = false;
        portENTER_CRITICAL(&g_mux);
        cmd_.mode = Mode::IDLE;
        portEXIT_CRITICAL(&g_mux);
        allowDrive = false;
        ul = ur = 0.0f;
        return;
    }

    const float u = D.duty / (float)PWM_MAX;
    if (D.wheel == 0) { ul = u;  ur = 0.0f; }
    else              { ul = 0.0f; ur = u; }
}

// ============================================================================
//  Odometry — docs/02 §3
// ============================================================================
void Robot::integrateOdometry(int32_t dL, int32_t dR) {
    const float dsL = (float)dL * DISTANCE_PER_COUNT_M;
    const float dsR = (float)dR * DISTANCE_PER_COUNT_M;

    const float ds  = 0.5f * (dsR + dsL);
    const float dth = (dsR - dsL) / WHEEL_TRACK_M;

#if ODOM_INTEGRATION == 1
    // ---- Euler: dùng θ CŨ ----
    pose_.x += ds * cosf(pose_.th);
    pose_.y += ds * sinf(pose_.th);

#elif ODOM_INTEGRATION == 3
    // ---- Cung tròn chính xác ----
    if (fabsf(dth) > 1e-6f) {
        const float r = ds / dth;
        pose_.x += r * ( sinf(pose_.th + dth) - sinf(pose_.th));
        pose_.y += r * (-cosf(pose_.th + dth) + cosf(pose_.th));
    } else {
        pose_.x += ds * cosf(pose_.th);
        pose_.y += ds * sinf(pose_.th);
    }

#else
    // ---- Mid-point / RK2 (KHUYẾN NGHỊ, sai số O(Δθ³)) ----
    const float thMid = pose_.th + 0.5f * dth;
    pose_.x += ds * cosf(thMid);
    pose_.y += ds * sinf(thMid);
#endif

    pose_.th = wrapToPi(pose_.th + dth);
}

// ============================================================================
//  Feedforward bù vùng chết — docs/03 §7
// ============================================================================
float Robot::computeFeedforward(float vsp, float dz) const {
    if (fabsf(vsp) < V_SP_DEADBAND) return 0.0f;
    const float s = (vsp > 0.0f) ? 1.0f : -1.0f;
    return s * (dz + fabsf(vsp) / par_.motorK);
}

// ============================================================================
//  Đọc điện áp pin — docs/01 §4.1
// ============================================================================
void Robot::readBattery() {
    uint32_t acc = 0;
    for (uint8_t i = 0; i < BATT_ADC_SAMPLES; i++) {
        acc += analogReadMilliVolts(PIN_BATT_ADC);
    }
    const float mv = (float)acc / (float)BATT_ADC_SAMPLES;
    vbat_ = (mv / 1000.0f) * BATT_DIVIDER_RATIO * BATT_CAL_GAIN;
}

// ============================================================================
//  Getters
// ============================================================================
Telemetry Robot::getTelemetry() {
    Telemetry t;
    portENTER_CRITICAL(&g_mux);
    t = tel_;
    portEXIT_CRITICAL(&g_mux);
    return t;
}

Mode Robot::mode() {
    Mode m;
    portENTER_CRITICAL(&g_mux);
    m = cmd_.mode;
    portEXIT_CRITICAL(&g_mux);
    return m;
}

uint16_t Robot::status()      { return safety_.bits(); }
uint16_t Robot::statusBits()  { return safety_.bits(); }
float    Robot::vbat()        { return vbat_; }
bool     Robot::imuOk()       { return imu_.ok() && imu_.calibrated(); }

Pose2D Robot::pose() {
    Pose2D p;
    portENTER_CRITICAL(&g_mux);
    p = pose_;
    portEXIT_CRITICAL(&g_mux);
    return p;
}

// ============================================================================
//  In trạng thái
// ============================================================================
void Robot::printStatus(Stream& s) {
    const Telemetry t = getTelemetry();
    s.printf("MODE=%s  PWM_L=%4d PWM_R=%4d  vL=%+.4f vR=%+.4f  v=%+.4f w=%+.4f\n",
             modeName(t.mode), t.pwm_l, t.pwm_r, t.vl, t.vr, t.v, t.w);
    s.printf("pose=(%+.4f, %+.4f, %+.2f deg)  CL=%lld CR=%lld\n",
             t.x, t.y, t.th * 180.0f / (float)M_PI,
             (long long)t.cl, (long long)t.cr);
    s.printf("VB=%.2fV  HZ=%.1f  JIT=%luus  IMU=%s(0x%02X, bz=%.4f dps)  ERR=0x%04X\n",
             t.vbat, t.hz, (unsigned long)t.jitter_us,
             t.imuOk ? "OK" : "LOI", t.whoAmI,
             t.imuBiasZ * 180.0f / (float)M_PI, t.status);
}

void Robot::printEncoders(Stream& s) {
    const float dL = (float)encL_.total() * DISTANCE_PER_COUNT_M;
    const float dR = (float)encR_.total() * DISTANCE_PER_COUNT_M;
    s.printf("CL=%lld CR=%lld   sL=%.4f m  sR=%.4f m   (chenh %.1f mm)\n",
             (long long)encL_.total(), (long long)encR_.total(),
             dL, dR, (dR - dL) * 1000.0f);
    s.printf("d_pc = %.6f mm/xung   N = %.1f xung/vong   %.1f xung/m\n",
             DISTANCE_PER_COUNT_M * 1000.0f, COUNTS_PER_WHEEL_REV, COUNTS_PER_METER);
}

void Robot::printImu(Stream& s) {
    const ImuData& d = imu_.data();
    s.printf("ACC[g]  x=%+.3f y=%+.3f z=%+.3f   |a|=%.3f\n",
             d.ax, d.ay, d.az, sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az));
    s.printf("GYRO    x=%+.4f y=%+.4f z=%+.4f rad/s  (%.2f %.2f %.2f dps)\n",
             d.gx, d.gy, d.gz,
             d.gx * 180.0f / (float)M_PI, d.gy * 180.0f / (float)M_PI, d.gz * 180.0f / (float)M_PI);
    s.printf("R/P/Y   roll=%+.2f pitch=%+.2f yaw=%+.2f deg   yaw_total=%+.1f deg\n",
             d.roll * 180.0f / (float)M_PI, d.pitch * 180.0f / (float)M_PI,
             d.yaw * 180.0f / (float)M_PI, imu_.yawTotal() * 180.0f / (float)M_PI);
    s.printf("TEMP=%.1f C   biasZ=%+.4f dps   whoAmI=0x%02X\n",
             d.tempC, imu_.biasZ() * 180.0f / (float)M_PI, imu_.whoAmI());
}

void Robot::printBattery(Stream& s) {
    const Telemetry t = getTelemetry();
    const float pct = constrain((t.vbat - 9.9f) / (12.6f - 9.9f) * 100.0f, 0.0f, 100.0f);
    s.printf("VBAT = %.2f V   (~%.0f%%)   [CANH BAO<%.1fV  CAT<%.1fV]\n",
             t.vbat, pct, BATT_WARN_V, BATT_CRIT_V);
    s.printf("He so chia ap: %.3f  |  BATT_CAL_GAIN: %.4f\n",
             BATT_DIVIDER_RATIO, BATT_CAL_GAIN);
}

void Robot::printConfig(Stream& s) {
    s.println(F("---------- CAU HINH HIEN TAI ----------"));
    s.printf("Co khi : D=%.1f mm  L=%.1f mm  N=%.1f  d_pc=%.6f mm\n",
             WHEEL_DIAMETER_M*1000.0f, WHEEL_TRACK_M*1000.0f,
             COUNTS_PER_WHEEL_REV, DISTANCE_PER_COUNT_M*1000.0f);
    s.printf("Dong hoc: v_max=%.4f  w_max=%.4f  v_safe=%.3f\n", V_MAX_MPS, W_MAX_RAD_S, V_SAFE_MPS);
    s.printf("PID L  : kp=%.4f ki=%.4f kd=%.4f\n", par_.kpL, par_.kiL, par_.kdL);
    s.printf("PID R  : kp=%.4f ki=%.4f kd=%.4f\n", par_.kpR, par_.kiR, par_.kdR);
    s.printf("FF     : motorK=%.4f ffGain=%.3f  DZ_L=%.4f DZ_R=%.4f  (mode=%d)\n",
             par_.motorK, par_.ffGain, par_.dzL, par_.dzR, PID_ANTIWINDUP_MODE);
    s.printf("Ramp   : accMax=%.3f m/s2  alphaMax=%.3f rad/s2\n", par_.accMax, par_.alphaMax);
    s.printf("Loc    : velTau=%.4f s   dFilterTau=%.4f s\n", par_.velFiltTau, PID_D_FILTER_TAU_S);
    s.printf("Huong  : Kp=%.3f Kd=%.3f  Wmax=%.3f\n", par_.headingKp, par_.headingKd, HEADING_W_MAX);
    s.printf("Goto   : Kdist=%.3f tol=%.3f vmax=%.3f\n", GOTO_KDIST, GOTO_TOL_M, GOTO_V_MAX);
    s.printf("Odom   : phuong phap %d (1=Euler 2=Midpoint 3=ExactArc)\n", ODOM_INTEGRATION);
    s.printf("An toan: cmdTimeout=%d ms  stallPWM>%.2f & |v|<%.2f trong %d ms\n",
             CMD_TIMEOUT_MS, STALL_PWM_THRESH, STALL_VEL_THRESH, STALL_TIME_MS);
    s.println(F("---------------------------------------"));
}

void Robot::printCsvHeader(Stream& s) {
    s.println(F("t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R"));
}

void Robot::printCsv(Stream& s) {
    const Telemetry t = getTelemetry();
    s.printf("%lu,%s,%.4f,%.4f,%.4f,%.4f,%d,%d\n",
             (unsigned long)t.t_ms, modeName(t.mode),
             t.vspL, t.vl, t.vspR, t.vr, t.pwm_l, t.pwm_r);
}
