/**
 * robot.h — "bộ não" của robot: cảm biến → odometry → bộ điều khiển → PWM.
 *
 * Chứa:
 *   • Động học vi sai thuận / nghịch              (docs/02 §2)
 *   • Tích phân odometry (Euler / mid-point / arc) (docs/02 §3)
 *   • PID vòng vận tốc 2 bánh + feedforward        (docs/03 §4, §7)
 *   • Bộ giữ hướng dùng rate-feedback từ gyro      (docs/02 §6, docs/03 §10.1)
 *   • Go-to-goal                                   (docs/03 §10.2)
 *   • Ramp vận tốc + giới hạn động học             (docs/02 §2.4)
 *   • Relay auto-tuning + các chế độ hiệu chuẩn    (docs/03 §6.2)
 *   • An toàn: kẹt, pin yếu, watchdog              (docs/04 §5)
 *
 * Mọi hàm cmd*() có thể gọi từ task comms (core 1, priority thấp).
 * step() chỉ được gọi từ controlTask (core 1, priority 10).
 * Truy cập trạng thái chia sẻ được bảo vệ bằng portMUX (xem robot.cpp).
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "imu.h"
#include "safety.h"

// ============================================================================
//  Kiểu dữ liệu
// ============================================================================
enum class Mode : uint8_t {
    IDLE = 0,   // dừng, coast
    DUTY,       // open-loop PWM trực tiếp
    WHEEL,      // PID vận tốc từng bánh
    BODY,       // (v, ω) của thân robot
    HEADING,    // đi thẳng, giữ hướng bằng gyro
    GOTO,       // tự hành tới điểm (x, y)
    SPIN,       // quay tại chỗ
    STEP,       // test step response (xuất CSV)
    RELAY,      // relay auto-tuning
    CAL_DZ      // quét tìm vùng chết PWM
};

const char* modeName(Mode m);

struct Pose2D {
    float x   = 0.0f;
    float y   = 0.0f;
    float th  = 0.0f;
};

struct Telemetry {
    uint32_t t_ms      = 0;
    float    x = 0, y = 0, th = 0;
    float    v = 0, w = 0;
    float    vl = 0, vr = 0;
    float    vspL = 0, vspR = 0;
    int64_t  cl = 0, cr = 0;
    float    ax = 0, ay = 0, az = 0;
    float    gx = 0, gy = 0, gz = 0;
    float    roll = 0, pitch = 0, yaw = 0;
    float    tempC = 0;
    int16_t  pwm_l = 0, pwm_r = 0;
    float    vbat  = 0;
    uint16_t status = 0;
    float    hz = 0;
    uint32_t jitter_us = 0;
    uint32_t seq = 0;
    Mode     mode = Mode::IDLE;
    bool     imuOk = false;
    uint8_t  whoAmI = 0;
    float    imuBiasZ = 0;
    // thành phần PID (để vẽ đồ thị, phân tích)
    float    pidPL = 0, pidIL = 0, pidDL = 0, pidFFL = 0;
    float    pidPR = 0, pidIR = 0, pidDR = 0, pidFFR = 0;
};

// ============================================================================
//  Robot
// ============================================================================
class Robot {
public:
    Robot();

    void begin();
    void step(float dt);                 // gọi từ controlTask

    // ---------------- Lệnh điều khiển (thread-safe) ----------------
    void cmdSetMode(Mode m, bool remote = false);
    void cmdOpenLoop(float ul, float ur, bool remote = false);
    void cmdWheel(float vl, float vr, bool remote = true);
    void cmdBody(float v, float w, bool remote = true);
    void cmdHeading(float v, float thRefRad, bool remote = false);
    void cmdGoto(float xg, float yg, bool remote = false);
    void cmdSpin(float w, bool remote = false);
    void cmdStepTest(float pwm, uint32_t ms);
    void cmdRelayTune(uint8_t wheel, float vsp);
    void cmdCalDeadzone(uint8_t wheel);

    void cmdStop(bool emergency = false);
    void cmdClearErrors();
    void cmdResetOdometry();
    void cmdResetYaw();
    /** Hiệu chuẩn bias gyro (robot phải đứng yên). Trả về false nếu thất bại. */
    bool cmdCalImu(uint16_t samples = IMU_CALIB_SAMPLES);

    /** Trả về true nếu tham số hợp lệ. */
    bool cmdSetGain(char wheel, const char* which, float value);
    bool cmdSetParam(const char* key, float value);

    void setImuFail();
    /** Bật/tắt một bit trạng thái bất kỳ (dùng bởi tầng comms cho cờ WiFi). */
    void setStatusBit(uint16_t bit, bool value);

    // ---------------- Đọc trạng thái (thread-safe) ----------------
    Telemetry getTelemetry();
    Mode      mode();
    uint16_t  status();
    uint16_t  statusBits();
    float     vbat();
    bool      imuOk();
    Pose2D    pose();
    void      setRemoteAlive(bool alive);

    // ---------------- In ra Stream ----------------
    void printStatus(Stream& s);
    void printEncoders(Stream& s);
    void printImu(Stream& s);
    void printBattery(Stream& s);
    void printConfig(Stream& s);
    void printCsvHeader(Stream& s);
    void printCsv(Stream& s);

    // ---------------- Tiện ích tĩnh ----------------
    static float wrapToPi(float a);

private:
    // --- Phần cứng ---
    EncoderPCNT    encL_;
    EncoderPCNT    encR_;
    Motor          motL_;
    Motor          motR_;
    PID            pidL_;
    PID            pidR_;
    Imu            imu_;
    SafetyMonitor  safety_;

    // --- Tham số có thể chỉnh online ---
    struct Params {
        float kpL = PID_KP_DEFAULT, kiL = PID_KI_DEFAULT, kdL = PID_KD_DEFAULT;
        float kpR = PID_KP_DEFAULT, kiR = PID_KI_DEFAULT, kdR = PID_KD_DEFAULT;
        float motorK   = MOTOR_K;
        float dzL      = DZ_FRACTION_L;
        float dzR      = DZ_FRACTION_R;
        float ffGain   = FF_GAIN;
        float accMax   = ACC_MAX_MPS2;
        float alphaMax = ALPHA_MAX_RADPS2;
        float headingKp = HEADING_KP;
        float headingKd = HEADING_KD;
        float velFiltTau = VEL_FILTER_TAU_S;
    } par_;

    // --- Lệnh hiện hành ---
    struct Cmd {
        Mode     mode        = Mode::IDLE;
        float    v = 0.0f, w = 0.0f;
        float    vl = 0.0f, vr = 0.0f;
        float    ul = 0.0f, ur = 0.0f;
        float    thRef = 0.0f;
        float    gx = 0.0f, gy = 0.0f;
        uint32_t lastRemoteMs = 0;
        bool     remoteSeen   = false;
        uint32_t stepPwm      = 0;
        uint32_t stepEndMs    = 0;
        uint8_t  relayWheel   = 0;
        float    relayVsp     = 0.15f;
        uint8_t  calWheel     = 0;
    } cmd_;

    // --- Trạng thái ---
    float    vL_ = 0.0f, vR_ = 0.0f;         // vận tốc đo được, đã lọc (m/s)
    float    vLraw_ = 0.0f, vRraw_ = 0.0f;
    bool     velFirst_ = true;
    float    vspL_ = 0.0f, vspR_ = 0.0f;     // setpoint sau slew (m/s)
    Pose2D   pose_;
    uint32_t lastImuUs_  = 0;
    uint32_t lastBattMs_ = 0;
    float    vbat_       = 0.0f;

    // --- Relay tuning ---
    struct RelayState {
        bool     active   = false;
        uint8_t  wheel    = 0;
        float    vsp      = 0.15f;
        int8_t   sign     = 1;
        float    tSwitch  = 0.0f;
        float    halfSum  = 0.0f;
        uint8_t  halfCnt  = 0;
        uint16_t switchCnt = 0;
        float    vMin     = 1e9f, vMax = -1e9f;
        bool     measuring = false;
        bool     done     = false;
    } relay_;

    // --- Hiệu chuẩn deadzone ---
    struct DzCal {
        bool    active = false;
        uint8_t wheel  = 0;
        float   duty   = 0.0f;
        bool    done   = false;
        float   result = 0.0f;
    } dz_;

    // --- Telemetry snapshot ---
    Telemetry tel_;
    uint32_t  telSeq_ = 0;
    float     hz_     = 0.0f;
    uint32_t  jitter_ = 0;

    // --- Hàm nội bộ ---
    void   computeWheelSetpoints(float dt, float& vspL, float& vspR, float& ulDirect, float& urDirect,
                                 bool& useDirect, bool& allowDrive);
    void   integrateOdometry(int32_t dL, int32_t dR);
    float  computeFeedforward(float vsp, float dz) const;
    void   readBattery();
    void   handleRelay(float dt, float& ulDirect, float& urDirect, bool& useDirect, bool& allowDrive);
    void   handleDeadzoneCal(float& ulDirect, float& urDirect, bool& useDirect, bool& allowDrive);
    void   resetControllers();
};
