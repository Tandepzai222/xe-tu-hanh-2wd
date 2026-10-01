#include "comms.h"
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const float RAD2DEG_F = 57.29577951f;

// ============================================================================
//  Khởi tạo
// ============================================================================
void Comms::begin(Robot* robot) {
    robot_ = robot;

    remoteIp_ = IPAddress();
    remoteIp_.fromString(LAPTOP_IP);
    if (remoteIp_ == IPAddress((uint32_t)0)) {
        Serial.printf("[NET] CANH BAO: LAPTOP_IP '%s' khong hop le. "
                      "Se dung IP cua goi lenh dau tien.\n", LAPTOP_IP);
    }

    // KHÔNG gọi udp.begin() ở đây — lwIP chưa sẵn sàng sẽ gây panic
    // "Invalid mbox" (tcpip_send_msg_wait_sem). Sẽ mở UDP sau khi WiFi connected.
    udpStarted_ = false;

    wifiStartMs_ = 0;
    connectWifi();

    Serial.println();
    Serial.println(F("Go `help` de xem danh sach lenh."));
    Serial.print(F("> "));
}

// ============================================================================
//  WiFi (không chặn)
// ============================================================================
void Comms::connectWifi() {
    const uint32_t now = millis();

    if (wifiStartMs_ == 0) {
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);              // ⚠️ giảm độ trễ gói tin đáng kể
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        wifiStartMs_    = now;
        lastWifiTryMs_  = now;
        Serial.printf("[NET] Dang ket noi WiFi '%s'...\n", WIFI_SSID);
        return;
    }

    if ((uint32_t)(now - wifiStartMs_) > WIFI_CONNECT_TIMEOUT_MS &&
        (uint32_t)(now - lastWifiTryMs_) > WIFI_RETRY_INTERVAL_MS) {
        lastWifiTryMs_ = now;
        WiFi.disconnect(true);
        delay(20);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        Serial.println(F("[NET] Thu ket noi lai WiFi..."));
    }
}

// ============================================================================
//  Vòng lặp chính (loopTask)
// ============================================================================
void Comms::loop() {
    const uint32_t now = millis();

    // ---------------- WiFi ----------------
    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected_) {
            wifiConnected_ = true;
            robot_->setStatusBit(ST_WIFI_DOWN, false);
            // Mở UDP SAU khi WiFi đã có IP — tránh panic "Invalid mbox" của lwIP
            if (!udpStarted_) {
                udp_.begin(UDP_PORT_CMD);
                udpStarted_ = true;
            }
            Serial.printf("\n[NET] Da ket noi. IP=%s  RSSI=%d dBm\n",
                          WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
            Serial.printf("[NET] Telemetry -> %s:%d   Lenh <- cong %d\n",
                          remoteIp_.toString().c_str(), UDP_PORT_TELEM, UDP_PORT_CMD);
            Serial.print(F("> "));
        }
    } else {
        if (wifiConnected_) {
            wifiConnected_ = false;
            if (udpStarted_) { udp_.stop(); udpStarted_ = false; }
            robot_->setStatusBit(ST_WIFI_DOWN, true);
            Serial.println(F("\n[NET] Mat ket noi WiFi!"));
            Serial.print(F("> "));
        }
        connectWifi();
    }

    // ---------------- Nhận UDP ----------------
    handleUdp();

    // ---------------- Nhận Serial ----------------
    handleSerial();

    // ---------------- Hiệu chuẩn L đang chạy ----------------
    updateSpinCal();

    // ---------------- Telemetry ----------------
    if (wifiConnected_ && (uint32_t)(now - lastTelemMs_) >= (uint32_t)TELEM_PERIOD_MS) {
        lastTelemMs_ = now;
        sendTelemetry();
    }

    // ---------------- Plot CSV qua Serial ----------------
    if (plotEnabled_ && (uint32_t)(now - lastPlotMs_) >= plotPeriodMs_) {
        lastPlotMs_ = now;
        robot_->printCsv(Serial);
    }

    // ---------------- LED trạng thái ----------------
    if ((uint32_t)(now - lastStatMs_) >= (uint32_t)STATUS_LED_PERIOD_MS) {
        lastStatMs_ = now;
        const uint16_t st = robot_->statusBits();
        const bool fatal  = (st & (ST_ESTOP | ST_STALL_L | ST_STALL_R | ST_BATT_LOW)) != 0;

        if (fatal) {
            digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));   // nháy nhanh
        } else if (!wifiConnected_) {
            static uint8_t slow = 0;
            digitalWrite(PIN_LED_STATUS, (++slow % 5) == 0);               // nháy chậm
        } else {
            digitalWrite(PIN_LED_STATUS, HIGH);                            // sáng liên tục = OK
        }
    }
}

// ============================================================================
//  UDP
// ============================================================================
void Comms::handleUdp() {
    if (!udpStarted_) return;
    int sz;
    while ((sz = udp_.parsePacket()) > 0) {
        const IPAddress src = udp_.remoteIP();

        if (sz > (int)sizeof(lineBuf_) - 8) {
            while (udp_.available()) udp_.read();
            parseErr_++;
            continue;
        }

        char buf[sizeof(lineBuf_)];
        const int n = udp_.read(buf, sizeof(buf) - 1);
        if (n <= 0) continue;
        buf[n] = '\0';
        rxCount_++;

        // Nếu chưa biết IP cấp trên → học từ gói vừa nhận
        if (remoteIp_ == IPAddress((uint32_t)0)) remoteIp_ = src;

        if (buf[0] == '{') processJson(buf);
        else               processTextCmd(buf);
    }
}

void Comms::sendRaw(const char* s) {
    if (!wifiConnected_ || !udpStarted_) return;
    if (remoteIp_ == IPAddress((uint32_t)0)) return; // chưa biết IP đích
    const size_t len = strlen(s);
    if (!udp_.beginPacket(remoteIp_, UDP_PORT_TELEM)) return; // ENOMEM -> bỏ gói này
    udp_.write((const uint8_t*)s, len);
    if (!udp_.endPacket()) return; // 12 = ENOMEM / WiFi bận -> bỏ qua, không đếm tx
    txCount_++;
}

void Comms::sendPong(uint32_t seq, uint32_t ts) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"c\":\"pong\",\"seq\":%lu,\"ts\":%lu,\"te\":%lu}\n",
             (unsigned long)seq, (unsigned long)ts, (unsigned long)millis());
    sendRaw(buf);
}

void Comms::sendTelemetry() {
    const Telemetry t = robot_->getTelemetry();

    char buf[640];
    const int n = snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"x\":%.4f,\"y\":%.4f,\"th\":%.4f,"
        "\"v\":%.4f,\"w\":%.4f,\"vl\":%.4f,\"vr\":%.4f,"
        "\"vspL\":%.4f,\"vspR\":%.4f,\"cl\":%lld,\"cr\":%lld,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,"
        "\"rp\":%.2f,\"pp\":%.2f,\"yw\":%.2f,"
        "\"pl\":%d,\"pr\":%d,\"vb\":%.2f,\"st\":%u,\"md\":\"%s\","
        "\"hz\":%.1f,\"jt\":%lu,\"seq\":%lu}\n",
        (unsigned long)t.t_ms, t.x, t.y, t.th,
        t.v, t.w, t.vl, t.vr,
        t.vspL, t.vspR, (long long)t.cl, (long long)t.cr,
        t.ax, t.ay, t.az,
        t.gx, t.gy, t.gz,
        t.roll * RAD2DEG_F, t.pitch * RAD2DEG_F, t.yaw * RAD2DEG_F,
        (int)t.pwm_l, (int)t.pwm_r, t.vbat, (unsigned)t.status, modeName(t.mode),
        t.hz, (unsigned long)t.jitter_us, (unsigned long)t.seq);

    if (n > 0 && n < (int)sizeof(buf)) sendRaw(buf);
}

// ============================================================================
//  Serial
// ============================================================================
void Comms::handleSerial() {
    while (Serial.available() > 0) {
        const char ch = (char)Serial.read();

        // 1. Phím tắt khẩn cấp: nếu đang plot mà bấm 'q' hoặc 'Q' -> Tắt ngay
        if (plotEnabled_ && (ch == 'q' || ch == 'Q')) {
            plotEnabled_ = false;
            lineLen_ = 0;
            Serial.println(F("\n[PLOT] Da dung nhanh bang phim 'q'."));
            Serial.print(F("> "));
            return;
        }

        // 2. Xử lý kết thúc dòng
        if (ch == '\r' || ch == '\n') {
            if (lineLen_ > 0) {
                lineBuf_[lineLen_] = '\0';
                processTextCmd(lineBuf_);
                lineLen_ = 0;
                if (!plotEnabled_) Serial.print(F("> "));
            }
        } 
        // 3. Xử lý Backspace (ASCII 8 hoặc 127) khi gõ nhầm
        else if (ch == '\b' || ch == 127) {
            if (lineLen_ > 0) {
                lineLen_--;
            }
        }
        // 4. Lưu ký tự bình thường
        else if (lineLen_ < sizeof(lineBuf_) - 1) {
            lineBuf_[lineLen_++] = ch;
        } else {
            lineLen_ = 0;
            Serial.println(F("[ERR] Dong lenh qua dai."));
        }
    }
}

void Comms::processLine(char* line, bool fromRemote) {
    (void)fromRemote;
    processTextCmd(line);
}

// ============================================================================
//  JSON (từ ROS 2)
// ============================================================================
void Comms::processJson(const char* payload) {
    StaticJsonDocument<512> doc;
    const DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        parseErr_++;
        Serial.printf("[UDP] JSON loi: %s\n", err.c_str());
        return;
    }

    const char* c = doc["c"] | "";
    if (c[0] == '\0') { parseErr_++; return; }

    if (!strcmp(c, "vel")) {
        robot_->cmdBody((float)(doc["v"] | 0.0f), (float)(doc["w"] | 0.0f), true);

    } else if (!strcmp(c, "wheel")) {
        robot_->cmdWheel((float)(doc["vl"] | 0.0f), (float)(doc["vr"] | 0.0f), true);

    } else if (!strcmp(c, "body")) {
        robot_->cmdBody((float)(doc["v"] | 0.0f), (float)(doc["w"] | 0.0f), true);

    } else if (!strcmp(c, "mode")) {
        const char* m = doc["m"] | "idle";
        if      (!strcmp(m, "idle"))    robot_->cmdSetMode(Mode::IDLE, true);
        else if (!strcmp(m, "heading")) robot_->cmdHeading((float)(doc["v"] | 0.2f),
                                                          (float)(doc["th"] | 0.0f), true);
        else if (!strcmp(m, "goto"))    robot_->cmdGoto((float)(doc["x"] | 0.0f),
                                                        (float)(doc["y"] | 0.0f), true);
        else if (!strcmp(m, "spin"))    robot_->cmdSpin((float)(doc["w"] | 0.0f), true);
        else Serial.printf("[UDP] mode khong ho tro: %s\n", m);

    } else if (!strcmp(c, "pid")) {
        const char* wd = doc["wd"] | "l";
        if (doc.containsKey("kp")) robot_->cmdSetGain(wd[0], "kp", (float)doc["kp"]);
        if (doc.containsKey("ki")) robot_->cmdSetGain(wd[0], "ki", (float)doc["ki"]);
        if (doc.containsKey("kd")) robot_->cmdSetGain(wd[0], "kd", (float)doc["kd"]);

    } else if (!strcmp(c, "cfg")) {
        const char* k = doc["k"] | "";
        if (k[0]) {
            const float val = (float)(doc["val"] | 0.0f);
            if (robot_->cmdSetParam(k, val)) nvsRemember(k, val);
        }

    } else if (!strcmp(c, "stop")) {
        robot_->cmdStop(false);

    } else if (!strcmp(c, "estop")) {
        robot_->cmdStop(true);

    } else if (!strcmp(c, "clear")) {
        robot_->cmdClearErrors();

    } else if (!strcmp(c, "reset_odom")) {
        robot_->cmdResetOdometry();

    } else if (!strcmp(c, "ping")) {
        sendPong((uint32_t)(doc["seq"] | (uint32_t)0), (uint32_t)(doc["ts"] | (uint32_t)0));

    } else {
        Serial.printf("[UDP] Lenh khong ho tro: %s\n", c);
    }
}

// ============================================================================
//  Lệnh text (Serial hoặc UDP dạng text)
// ============================================================================
void Comms::processTextCmd(char* line) {
    char* save = nullptr;
    char* cmd  = strtok_r(line, " \t\r\n", &save);
    if (cmd == nullptr) return;

    auto nextS = [&]() -> char* { return strtok_r(nullptr, " \t\r\n", &save); };
    auto nextF = [&]() -> float { char* t = nextS(); return t ? atof(t) : 0.0f; };

    // ---------------------------------------------------------------- help
    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printHelp(Serial);
        return;
    }

    // -------------------------------------------------------------- status
    if (!strcmp(cmd, "status"))  { robot_->printStatus(Serial);   return; }
    if (!strcmp(cmd, "enc") || !strcmp(cmd, "encoder")) {
        char* a = nextS();
        if (a && !strcmp(a, "reset")) robot_->cmdResetOdometry();
        else                          robot_->printEncoders(Serial);
        return;
    }
    if (!strcmp(cmd, "odom"))    { robot_->printStatus(Serial);   return; }
    if (!strcmp(cmd, "imu"))     { robot_->printImu(Serial);      return; }
    if (!strcmp(cmd, "batt"))    { robot_->printBattery(Serial);  return; }
    if (!strcmp(cmd, "cfg")) {
        char* k = nextS();
        if (!k) { robot_->printConfig(Serial); return; }
        const float val = nextF();
        if (robot_->cmdSetParam(k, val)) nvsRemember(k, val);
        else Serial.printf("[CFG] Tham so khong hop le: %s\n", k);
        return;
    }

    // ---------------------------------------------------------------- plot
    if (!strcmp(cmd, "plot")) {
        char* a = nextS();
        if (!a || !strcmp(a, "off") || !strcmp(a, "0")) {
            plotEnabled_ = false;
            Serial.println(F("[PLOT] Tat."));
        } else {
            const float hz = atof(a);
            if (hz >= 1.0f && hz <= 200.0f) {
                plotPeriodMs_ = (uint32_t)(1000.0f / hz);
                plotEnabled_  = true;
                Serial.println(F("t_ms,md,vspL,vL,vspR,vR,pwm_L,pwm_R"));
            } else {
                Serial.println(F("[PLOT] Tan so phai trong 1..200 Hz."));
            }
        }
        return;
    }

    // ---------------------------------------------------------------- mode
    if (!strcmp(cmd, "mode")) {
        char* sub = nextS();
        if (!sub) { Serial.printf("[MODE] Hien tai: %s\n", modeName(robot_->mode())); return; }

        if (!strcmp(sub, "idle")) {
            robot_->cmdSetMode(Mode::IDLE);
        } else if (!strcmp(sub, "duty")) {
            robot_->cmdOpenLoop(nextF() / (float)PWM_MAX, nextF() / (float)PWM_MAX);
        } else if (!strcmp(sub, "wheel")) {
            robot_->cmdWheel(nextF(), nextF(), false);
        } else if (!strcmp(sub, "body")) {
            robot_->cmdBody(nextF(), nextF(), false);
        } else if (!strcmp(sub, "heading")) {
            const float v  = nextF();
            char* d = nextS();
            const float th = d ? (atof(d) * (float)DEG_TO_RAD) : 0.0f;
            if (!d) robot_->cmdResetYaw();
            robot_->cmdHeading(v, th);
        } else if (!strcmp(sub, "spin")) {
            robot_->cmdSpin(nextF());
        } else if (!strcmp(sub, "goto")) {
            robot_->cmdGoto(nextF(), nextF());
        } else if (!strcmp(sub, "step")) {
            robot_->cmdStepTest(nextF(), (uint32_t)nextF());
        } else if (!strcmp(sub, "relay")) {
            char* wd = nextS();
            robot_->cmdRelayTune((wd && wd[0] == 'r') ? 1 : 0, nextF());
        } else if (!strcmp(sub, "cal_dz")) {
            char* wd = nextS();
            robot_->cmdCalDeadzone((wd && wd[0] == 'r') ? 1 : 0);
        } else {
            Serial.printf("[MODE] Khong ho tro: %s\n", sub);
        }
        return;
    }

    // ------------------------------------------------------- lệnh tắt ngắn
    if (!strcmp(cmd, "duty"))   { robot_->cmdOpenLoop(nextF()/(float)PWM_MAX, nextF()/(float)PWM_MAX); return; }
    if (!strcmp(cmd, "wheel"))  { robot_->cmdWheel(nextF(), nextF(), false); return; }
    if (!strcmp(cmd, "body"))   { robot_->cmdBody(nextF(), nextF(), false);  return; }
    if (!strcmp(cmd, "spin"))   { robot_->cmdSpin(nextF());                  return; }
    if (!strcmp(cmd, "goto"))   { robot_->cmdGoto(nextF(), nextF());         return; }
    if (!strcmp(cmd, "step"))   { robot_->cmdStepTest(nextF(), (uint32_t)nextF()); return; }
    if (!strcmp(cmd, "heading")) {
        const float v = nextF();
        char* d = nextS();
        if (d) robot_->cmdHeading(v, atof(d) * (float)DEG_TO_RAD);
        else   { robot_->cmdResetYaw(); robot_->cmdHeading(v, 0.0f); }
        return;
    }
    if (!strcmp(cmd, "relay")) {
        char* wd = nextS();
        robot_->cmdRelayTune((wd && wd[0] == 'r') ? 1 : 0, nextF());
        return;
    }

    // ----------------------------------------------------------------- pid
    if (!strcmp(cmd, "pid")) {
        char* wd = nextS();
        char* which = nextS();
        const float val = nextF();
        if (!wd || !which) {
            Serial.println(F("[PID] Cu phap: pid <l|r|h> <kp|ki|kd> <gia tri>"));
            return;
        }
        if (robot_->cmdSetGain(wd[0], which, val)) {
            char key[20];
            snprintf(key, sizeof(key), "pid:%c:%s", wd[0], which);
            nvsRemember(key, val);
        }
        return;
    }

    // ----------------------------------------------------------------- cal
    if (!strcmp(cmd, "cal")) {
        char* what = nextS();
        if (!what) {
            Serial.println(F("[CAL] cal imu | cal deadzone <l|r> | cal batt <V_thuc> | cal enc"));
            return;
        }
        if (!strcmp(what, "imu")) {
            robot_->cmdCalImu();
        } else if (!strcmp(what, "deadzone")) {
            char* wd = nextS();
            robot_->cmdCalDeadzone((wd && wd[0] == 'r') ? 1 : 0);
        } else if (!strcmp(what, "batt")) {
            const float vtrue = nextF();
            const float vmeas = robot_->vbat();
            if (vmeas > 1.0f && vtrue > 1.0f) {
                Serial.printf("[CAL] VBAT do duoc = %.3f V, dong ho = %.3f V\n", vmeas, vtrue);
                Serial.printf("[CAL] => BATT_CAL_GAIN moi = %.4f  (sua trong config.h)\n",
                              BATT_CAL_GAIN * vtrue / vmeas);
            } else {
                Serial.println(F("[CAL] Do duoc dien ap khong hop le."));
            }
        } else if (!strcmp(what, "enc")) {
            robot_->cmdResetOdometry();
            Serial.println(F("[CAL] Da reset. Hay DAY TAY robot dung 3.000 m theo duong thang,"));
            Serial.println(F("[CAL] sau do go `enc` va tinh: d_pc = 3.000 / (so xung trung binh)"));
        } else if (!strcmp(what, "spin")) {
            char* n = nextS();
            const uint8_t turns = (uint8_t)(n ? atoi(n) : 5);
            startSpinCal(turns == 0 ? 5 : turns);
        } else {
            Serial.printf("[CAL] Khong ho tro: %s\n", what);
        }
        return;
    }

    // ------------------------------------------------------- an toàn / reset
    if (!strcmp(cmd, "stop"))  { robot_->cmdStop(false); return; }
    if (!strcmp(cmd, "estop")) { robot_->cmdStop(true);  return; }
    if (!strcmp(cmd, "clear") || !strcmp(cmd, "reset")) { robot_->cmdClearErrors(); return; }
    if (!strcmp(cmd, "reset_odom") || !strcmp(cmd, "reset_enc")) { robot_->cmdResetOdometry(); return; }
    if (!strcmp(cmd, "reset_yaw")) { robot_->cmdResetYaw(); return; }

    // ---------------------------------------------------------------- NVS
    if (!strcmp(cmd, "save")) { nvsSave(); return; }
    if (!strcmp(cmd, "load")) { nvsLoad(); return; }

    // --------------------------------------------------------------- WiFi
    if (!strcmp(cmd, "wifi")) {
        char* a = nextS();
        if (a && !strcmp(a, "reconnect")) {
            WiFi.disconnect(true);
            delay(50);
            wifiStartMs_ = 0;
            wifiConnected_ = false;
            connectWifi();
        } else if (a && !strcmp(a, "ip")) {
            Serial.printf("[NET] IP = %s\n", WiFi.localIP().toString().c_str());
        } else {
            printWifi(Serial);
        }
        return;
    }

    // --------------------------------------------------------------- debug
    if (!strcmp(cmd, "reboot")) { Serial.println(F("[SYS] Khoi dong lai...")); delay(100); ESP.restart(); return; }

    Serial.printf("[CMD] Lenh khong hop le: '%s'  (go `help`)\n", cmd);
}

// ============================================================================
//  In thông tin
// ============================================================================
void Comms::printWifi(Stream& s) {
    s.printf("WiFi: %s\n", wifiConnected_ ? "DA KET NOI" : "CHUA KET NOI");
    s.printf("  SSID     : %s\n", WIFI_SSID);
    s.printf("  IP robot : %s\n", WiFi.localIP().toString().c_str());
    s.printf("  RSSI     : %d dBm  (%s)\n", (int)WiFi.RSSI(),
             WiFi.RSSI() > -60 ? "tot" : (WiFi.RSSI() > -75 ? "trung binh" : "yeu"));
    s.printf("  Gui telemetry toi : %s:%d\n", remoteIp_.toString().c_str(), UDP_PORT_TELEM);
    s.printf("  Nhan lenh tren cong: %d\n", UDP_PORT_CMD);
    s.printf("  Da gui %lu goi, da nhan %lu goi, loi parse %lu\n",
             (unsigned long)txCount_, (unsigned long)rxCount_, (unsigned long)parseErr_);
}

void Comms::printHelp(Stream& s) {
    s.println();
    s.println(F("================= DANH SACH LENH ================="));
    s.println(F("--- Che do chay ---"));
    s.println(F("  mode idle                     dung, PWM = 0"));
    s.println(F("  mode duty <L> <R>             open-loop, PWM -1023..1023"));
    s.println(F("  mode wheel <vL> <vR>          PID van toc tung banh (m/s)"));
    s.println(F("  mode body <v> <w>             van toc than robot (m/s, rad/s)"));
    s.println(F("  mode heading <v> [goc_do]     di thang, giu huong bang IMU"));
    s.println(F("  mode spin <w>                 quay tai cho (rad/s)"));
    s.println(F("  mode goto <x> <y>             tu hanh toi diem (m)"));
    s.println(F("  mode step <pwm> <ms>          test step response (xuat CSV)"));
    s.println(F("  mode relay <l|r> <v_sp>       relay auto-tuning"));
    s.println(F("  mode cal_dz <l|r>             quet tim vung chet PWM"));
    s.println(F("--- Doc du lieu ---"));
    s.println(F("  status | enc [reset] | odom | imu | batt | cfg"));
    s.println(F("  plot <hz> | plot off          xuat CSV lien tuc qua Serial"));
    s.println(F("--- Chinh tham so ---"));
    s.println(F("  pid <l|r|h> <kp|ki|kd> <v>"));
    s.println(F("  cfg <key> <v>    key: motor_k dz dzl dzr ff_gain acc_max"));
    s.println(F("                        alpha_max vel_tau heading_kp heading_kd"));
    s.println(F("--- Hieu chuan ---"));
    s.println(F("  cal imu                       hieu chuan bias gyro (DUNG YEN!)"));
    s.println(F("  cal deadzone <l|r>            tim vung chet PWM"));
    s.println(F("  cal enc                       huong dan do d_pc bang 3 m"));
    s.println(F("  cal spin <so_vong>            quay tai cho -> hieu chuan L"));
    s.println(F("  cal batt <V_thuc>             tinh BATT_CAL_GAIN"));
    s.println(F("--- An toan / khac ---"));
    s.println(F("  stop | estop | clear | reset_odom | reset_yaw"));
    s.println(F("  save | load                   luu/nap tham so vao NVS"));
    s.println(F("  wifi [reconnect|ip] | reboot | help"));
    s.println(F("=================================================="));
}

// ============================================================================
//  Hiệu chuẩn L — quay tại chỗ N vòng, so odometry với IMU   (docs/02 §7.2)
//
//  Nguyên lý: ω_odom = (vR − vL)/L  →  θ_odom ∝ 1/L
//             Nếu θ_odom lệch θ_imu thì L đúng phải là:
//                 L_mới = L_cũ × (θ_odom / θ_imu)
// ============================================================================
void Comms::startSpinCal(uint8_t revolutions) {
    const Telemetry t = robot_->getTelemetry();

    if (!t.imuOk) {
        Serial.println(F("[CAL] Khong the hieu chuan: IMU chua hieu chuan xong."));
        return;
    }

    spinCalTurns_  = revolutions;
    spinCalYaw0_   = t.yaw;              // rad
    spinCalOdom0_  = t.th;               // rad
    spinCalTarget_ = 2.0f * (float)M_PI * (float)revolutions;
    spinCalActive_ = true;
    spinCalStartMs_ = millis();

    robot_->cmdResetOdometry();
    robot_->cmdSpin(1.0f);               // 1 rad/s ≈ 6 vòng/phút, đủ chậm để IMU theo kịp

    Serial.println();
    Serial.printf("[CAL] Bat dau quay %u vong voi omega = 1.0 rad/s.\n", revolutions);
    Serial.printf("[CAL] Du kien mat ~%.0f giay. Dat robot cho trong, khong vuong day.\n",
                  spinCalTarget_ / 1.0f);
    Serial.println(F("[CAL] Go `stop` de huy."));
}

void Comms::updateSpinCal() {
    if (!spinCalActive_) return;

    const Telemetry t = robot_->getTelemetry();
    const float dyaw = t.yaw - spinCalYaw0_;

    // Nếu người dùng đã bấm stop hoặc đổi chế độ → huỷ
    if (robot_->mode() != Mode::SPIN) {
        spinCalActive_ = false;
        Serial.println(F("[CAL] Da huy hieu chuan L."));
        return;
    }

    // Hết thời gian an toàn (gấp 3 lần dự kiến)
    if ((uint32_t)(millis() - spinCalStartMs_) > (uint32_t)(spinCalTarget_ * 3000.0f + 5000.0f)) {
        spinCalActive_ = false;
        robot_->cmdStop(false);
        Serial.println(F("[CAL] Het thoi gian cho. Da dung robot."));
        return;
    }

    if (fabsf(dyaw) < spinCalTarget_) return;

    // ---------------- Xong: dừng và tính L ----------------
    robot_->cmdStop(false);
    spinCalActive_ = false;

    const float d_odom = t.th - spinCalOdom0_;
    const float d_imu  = dyaw;

    Serial.println();
    Serial.println("=========== KET QUA HIEU CHUAN L ===========");
    Serial.printf("So vong quay        : %u\n", spinCalTurns_);
    Serial.printf("Goc theo ODOMETRY   : %+.2f do\n", d_odom * 180.0f / (float)M_PI);
    Serial.printf("Goc theo IMU        : %+.2f do\n", d_imu  * 180.0f / (float)M_PI);

    if (fabsf(d_imu) < 1e-3f) {
        Serial.println("Loi: goc IMU ~ 0, khong tinh duoc.");
        Serial.println("============================================");
        return;
    }

    const float err = (d_odom - d_imu) / d_imu * 100.0f;
    const float Lnew = WHEEL_TRACK_M * (d_odom / d_imu);
    const bool  flip = (d_odom * d_imu) < 0.0f;

    Serial.printf("Sai so hien tai     : %+.2f %%\n", err);
    Serial.println("--------------------------------------------");

    if (flip) {
        Serial.println("LOI: hai goc NGUOC DAU nhau -> encoder bi dao chieu!");
        Serial.println("     Kiem tra ENC_L_INVERT / ENC_R_INVERT trong config.h");
    } else {
        Serial.printf("L hien tai          : %.2f mm\n", WHEEL_TRACK_M * 1000.0f);
        Serial.printf("=> L MOI            : %.2f mm   (%.4f m)\n", Lnew * 1000.0f, Lnew);
        Serial.println();
        Serial.printf("Nap: cfg khong ho tro truc tiep -> sua config.h:\n");
        Serial.printf("     #define WHEEL_TRACK_M   %.4ff\n", Lnew);
        Serial.println("     Va cap nhat CUNG gia tri trong URDF (xe_tu_hanh.urdf.xacro)!");
        Serial.println();
        if (fabsf(err) < 2.0f) {
            Serial.println("=> DAT: sai so < 2%. Khong can chinh.");
        } else {
            Serial.println("=> CAN CHINH: sai so >= 2%. Nap L moi roi do lai de xac nhan.");
        }
    }
    Serial.println("============================================");
}

// ============================================================================
//  NVS — lưu tham số đã chỉnh online (config.h vẫn là nguồn chân lý)
// ============================================================================
void Comms::nvsRemember(const char* key, float value) {
    for (uint8_t i = 0; i < nvsCount_; i++) {
        if (!strcmp(nvsEntries_[i].key, key)) { nvsEntries_[i].val = value; return; }
    }
    if (nvsCount_ < NVS_MAX) {
        strncpy(nvsEntries_[nvsCount_].key, key, sizeof(nvsEntries_[0].key) - 1);
        nvsEntries_[nvsCount_].key[sizeof(nvsEntries_[0].key) - 1] = '\0';
        nvsEntries_[nvsCount_].val = value;
        nvsCount_++;
    } else {
        Serial.println(F("[NVS] Bang ghi da day (24 muc)."));
    }
}

void Comms::nvsSave() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println(F("[NVS] Khong mo duoc namespace."));
        return;
    }
    prefs.putUChar("ver", CONFIG_VERSION);
    prefs.putUChar("n", nvsCount_);
    for (uint8_t i = 0; i < nvsCount_; i++) {
        char k[24];
        snprintf(k, sizeof(k), "k%u", (unsigned)i);
        prefs.putString(k, nvsEntries_[i].key);
        snprintf(k, sizeof(k), "v%u", (unsigned)i);
        prefs.putFloat(k, nvsEntries_[i].val);
    }
    prefs.end();
    Serial.printf("[NVS] Da luu %u tham so.\n", (unsigned)nvsCount_);
}

void Comms::nvsLoad() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println(F("[NVS] Chua co du lieu da luu."));
        return;
    }
    const uint8_t ver = prefs.getUChar("ver", 0);
    if (ver != CONFIG_VERSION) {
        prefs.end();
        Serial.printf("[NVS] Phien ban cau hinh khac (%u != %d). Bo qua.\n", ver, CONFIG_VERSION);
        return;
    }
    const uint8_t n = prefs.getUChar("n", 0);
    uint8_t applied = 0;
    for (uint8_t i = 0; i < n && i < NVS_MAX; i++) {
        char k[24];
        snprintf(k, sizeof(k), "k%u", (unsigned)i);
        const String key = prefs.getString(k, "");
        snprintf(k, sizeof(k), "v%u", (unsigned)i);
        const float val = prefs.getFloat(k, 0.0f);
        if (key.length() == 0) continue;

        bool ok = false;
        if (key.startsWith("pid:")) {
            // dinh dang "pid:l:kp"
            const char wd    = key.charAt(4);
            const String w   = key.substring(6);
            ok = robot_->cmdSetGain(wd, w.c_str(), val);
        } else {
            ok = robot_->cmdSetParam(key.c_str(), val);
        }
        if (ok) { nvsRemember(key.c_str(), val); applied++; }
    }
    prefs.end();
    Serial.printf("[NVS] Da nap %u tham so.\n", (unsigned)applied);
}
