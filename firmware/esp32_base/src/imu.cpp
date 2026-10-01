#include "imu.h"
#include <Wire.h>
#include <math.h>

// ---- Thanh ghi MPU-60x0 / 6500 ----
static const uint8_t REG_SMPLRT_DIV     = 0x19;
static const uint8_t REG_CONFIG         = 0x1A;
static const uint8_t REG_GYRO_CONFIG    = 0x1B;
static const uint8_t REG_ACCEL_CONFIG   = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2  = 0x1D;
static const uint8_t REG_INT_PIN_CFG    = 0x37;
static const uint8_t REG_ACCEL_XOUT_H   = 0x3B;
static const uint8_t REG_PWR_MGMT_1     = 0x6B;
static const uint8_t REG_PWR_MGMT_2     = 0x6C;
static const uint8_t REG_WHO_AM_I       = 0x75;

static inline float wrapToPiF(float a) {
    a = fmodf(a + (float)M_PI, 2.0f * (float)M_PI);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    return a - (float)M_PI;
}

// ============================================================================
//  I²C helpers
// ============================================================================
bool Imu::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool Imu::readRegs(uint8_t reg, uint8_t* buf, size_t n) {
    Wire.beginTransmission(IMU_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)IMU_I2C_ADDR, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

// ============================================================================
//  begin()
// ============================================================================
bool Imu::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, IMU_I2C_FREQ);

    // --- Dò WHO_AM_I ---
    uint8_t who = 0;
    if (!readRegs(REG_WHO_AM_I, &who, 1)) {
        Serial.println("[IMU] LOI: khong doc duoc thanh ghi WHO_AM_I (kiem tra SDA/SCL/nguon).");
        ok_ = false;
        return false;
    }
    whoAmI_ = who;

    // Các mã chấp nhận được (bản đồ thanh ghi accel/gyro giống nhau):
    //   0x68 MPU-6050/6000/9150 | 0x70 MPU-6500 | 0x71/0x73 MPU-9250
    //   0x19 MPU-6886           | 0x98 ICM-20602
    const bool known = (who == 0x68 || who == 0x70 || who == 0x71 ||
                        who == 0x73 || who == 0x19 || who == 0x98);
    Serial.printf("[IMU] WHO_AM_I = 0x%02X%s\n", who,
                  known ? "" : "  (CANH BAO: ma la, kiem tra lai bang docs/01 muc 5.1)");

    // --- Reset thiết bị ---
    writeReg(REG_PWR_MGMT_1, 0x80);
    delay(120);
    writeReg(REG_PWR_MGMT_1, 0x01);   // clock = PLL trục X gyro (ổn định nhất)
    delay(20);
    writeReg(REG_PWR_MGMT_2, 0x00);   // bật cả 6 trục

    // --- Cấu hình ---
    writeReg(REG_SMPLRT_DIV,   IMU_SMPLRT_DIV);
    writeReg(REG_CONFIG,       IMU_DLPF_CFG);
    writeReg(REG_GYRO_CONFIG,  IMU_GYRO_FS_SEL);
    writeReg(REG_ACCEL_CONFIG, IMU_ACCEL_AFS_SEL);
    writeReg(REG_ACCEL_CONFIG2, 0x00);          // accel DLPF = cùng cấu hình gyro
    writeReg(REG_INT_PIN_CFG,  0x02);           // bypass enable (cho magnetometer sau này)
    delay(20);

    ok_ = true;
    Serial.printf("[IMU] San sang. Gyro ±500 dps (%.1f LSB/dps), Accel ±2g (%.0f LSB/g)\n",
                  IMU_GYRO_SENS, IMU_ACCEL_SENS);
    return true;
}

// ============================================================================
//  Hiệu chuẩn bias gyro — 2 lượt có loại outlier
// ============================================================================
bool Imu::readGyroZRaw(int16_t& raw) {
    uint8_t b[14];
    if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;
    raw = (int16_t)((b[12] << 8) | b[13]);
    return true;
}

bool Imu::calibrateGyro(uint16_t samples) {
    if (!ok_) return false;
    if (samples < 50) samples = 50;

    Serial.printf("[IMU] Hieu chuan bias gyro: %u mau (ROBOT PHAI DUNG YEN)...\n", samples);

    const float lsbToRad = (float)(M_PI / 180.0) / IMU_GYRO_SENS;

    // ---------------- Lượt 1: Welford để có mean & std ----------------
    uint32_t n = 0;
    double   mean = 0.0, m2 = 0.0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t raw;
        if (!readGyroZRaw(raw)) { delay(2); continue; }
        const double x = (double)raw * lsbToRad;
        n++;
        const double delta = x - mean;
        mean += delta / (double)n;
        m2   += delta * (x - mean);
    }
    if (n < 50) {
        Serial.println("[IMU] LOI: khong doc duoc du mau. Kiem tra I2C.");
        return false;
    }
    const double var = m2 / (double)(n - 1);
    const double sd  = sqrt(var);
    stdZ_ = (float)sd;

    Serial.printf("[IMU] Luot 1: n=%u  mean=%.5f rad/s (%.4f dps)  std=%.5f rad/s (%.4f dps)\n",
                  (unsigned)n, mean, mean * 180.0 / M_PI, sd, sd * 180.0 / M_PI);

    if (sd > IMU_CALIB_MAX_STD_RADPS) {
        Serial.printf("[IMU] CANH BAO: std qua lon (> %.4f rad/s) -> robot dang RUNG "
                      "hoac co ai cham vao ban. Hay dat robot dung yen va chay lai `cal imu`.\n",
                      IMU_CALIB_MAX_STD_RADPS);
    }

    // ---------------- Lượt 2: loại outlier > 3σ ----------------
    const double lo = mean - 3.0 * sd;
    const double hi = mean + 3.0 * sd;
    double sum = 0.0; uint32_t kept = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t raw;
        if (!readGyroZRaw(raw)) { delay(2); continue; }
        const double x = (double)raw * lsbToRad;
        if (x >= lo && x <= hi) { sum += x; kept++; }
    }
    if (kept > 20) {
        biasZ_ = (float)(sum / (double)kept);
    } else {
        biasZ_ = (float)mean;
        Serial.println("[IMU] Luot 2: khong du mau hop le, dung gia tri luot 1.");
    }

    // Bias trục X/Y: chỉ dùng lượt 1 (không quan trọng bằng Z cho robot phẳng)
    biasX_ = 0.0f;
    biasY_ = 0.0f;

    calibrated_ = true;
    Serial.printf("[IMU] => biasZ = %.5f rad/s (%.4f dps)   [n_kept=%u]\n",
                  biasZ_, biasZ_ * 180.0f / (float)M_PI, (unsigned)kept);
    Serial.println("[IMU] Ghi so nay vao bang so lieu (docs/02 muc 8).");
    return true;
}

// ============================================================================
//  poll() — đọc mẫu + lọc bù + tích phân yaw
// ============================================================================
void Imu::poll(float dt) {
    if (!ok_) return;
    if (dt <= 1e-6f) dt = 0.005f;

    uint8_t b[14];
    if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return;

    const int16_t rawAx = (int16_t)((b[0]  << 8) | b[1]);
    const int16_t rawAy = (int16_t)((b[2]  << 8) | b[3]);
    const int16_t rawAz = (int16_t)((b[4]  << 8) | b[5]);
    const int16_t rawT  = (int16_t)((b[6]  << 8) | b[7]);
    const int16_t rawGx = (int16_t)((b[8]  << 8) | b[9]);
    const int16_t rawGy = (int16_t)((b[10] << 8) | b[11]);
    const int16_t rawGz = (int16_t)((b[12] << 8) | b[13]);

    const float dpsToRad = (float)(M_PI / 180.0);

    d_.ax = IMU_AX_SIGN * (float)rawAx / IMU_ACCEL_SENS;
    d_.ay = IMU_AY_SIGN * (float)rawAy / IMU_ACCEL_SENS;
    d_.az = IMU_AZ_SIGN * (float)rawAz / IMU_ACCEL_SENS;
    d_.tempC = (float)rawT / 340.0f + 36.53f;

    d_.gx = IMU_GX_SIGN * ((float)rawGx / IMU_GYRO_SENS) * dpsToRad - biasX_;
    d_.gy = IMU_GY_SIGN * ((float)rawGy / IMU_GYRO_SENS) * dpsToRad - biasY_;
    d_.gz = IMU_GZ_SIGN * ((float)rawGz / IMU_GYRO_SENS) * dpsToRad - biasZ_;

    // ---------------- Complementary filter cho roll / pitch ----------------
    // τ suy ra từ IMU_COMP_ALPHA tại dt danh định 5 ms (xem docs/02 §5.5)
    const float tauNom  = IMU_COMP_ALPHA * 0.005f / (1.0f - IMU_COMP_ALPHA);
    const float alpha   = tauNom / (tauNom + dt);

    const float rollAcc  = atan2f(d_.ay, d_.az);
    const float pitchAcc = atan2f(-d_.ax, sqrtf(d_.ay * d_.ay + d_.az * d_.az));

    if (firstPoll_) {
        d_.roll  = rollAcc;
        d_.pitch = pitchAcc;
        prevGz_  = d_.gz;
        firstPoll_ = false;
    } else {
        d_.roll  = alpha * (d_.roll  + d_.gx * dt) + (1.0f - alpha) * rollAcc;
        d_.pitch = alpha * (d_.pitch + d_.gy * dt) + (1.0f - alpha) * pitchAcc;
    }

    // ---------------- Yaw: tích phân hình thang (trapezoidal) ----------------
    // ⚠️ KHÔNG có magnetometer → bias còn sót sẽ làm yaw trôi.
    //    Bù lại bằng EKF trên ROS 2 (docs/05 §6).
    const float dyaw = 0.5f * (d_.gz + prevGz_) * dt;
    prevGz_    = d_.gz;
    yawTotal_ += dyaw;
    d_.yaw     = wrapToPiF(yawTotal_);
}
