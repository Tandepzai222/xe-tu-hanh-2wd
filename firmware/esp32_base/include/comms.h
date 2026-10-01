/**
 * comms.h — WiFi UDP (giao thức với ROS 2) + console Serial.
 *
 * Đặc tả giao thức: docs/05_giao-thuc-va-ros2.md §2
 *
 * Chạy trong loopTask (core 1, priority 1). Có thể bị block bởi WiFi —
 * đó là lý do vòng PID nằm ở controlTask riêng (docs/04 §2).
 */
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "config.h"
#include "robot.h"

class Comms {
public:
    void begin(Robot* robot);
    void loop();

    bool  wifiConnected() const { return wifiConnected_; }
    int   rssi() const { return (int)WiFi.RSSI(); }
    IPAddress ip() const { return WiFi.localIP(); }

    void printWifi(Stream& s);
    void printHelp(Stream& s);

private:
    // ---------------- WiFi / UDP ----------------
    void connectWifi();
    void handleUdp();
    void sendTelemetry();
    void sendRaw(const char* s);
    void sendPong(uint32_t seq, uint32_t ts);

    // ---------------- Lệnh ----------------
    void handleSerial();
    void processLine(char* line, bool fromRemote);
    void processJson(const char* payload);
    void processTextCmd(char* line);

    // ---------------- Hiệu chuẩn L (quay tại chỗ) ----------------
    void startSpinCal(uint8_t revolutions);
    void updateSpinCal();

    // ---------------- NVS ----------------
    void nvsRemember(const char* key, float value);
    void nvsSave();
    void nvsLoad();

    Robot*    robot_ = nullptr;

    WiFiUDP   udp_;
    bool      wifiConnected_   = false;
    bool      udpStarted_      = false;
    uint32_t  lastWifiTryMs_   = 0;
    uint32_t  wifiStartMs_     = 0;
    IPAddress remoteIp_;

    // ---------------- Serial ----------------
    char      lineBuf_[256] = {0};
    uint16_t  lineLen_      = 0;

    // ---------------- Thời gian ----------------
    uint32_t  lastTelemMs_  = 0;
    uint32_t  lastStatMs_   = 0;

    // ---------------- Chế độ plot CSV ----------------
    bool      plotEnabled_  = false;
    uint32_t  plotPeriodMs_ = 20;
    uint32_t  lastPlotMs_   = 0;

    // ---------------- Thống kê ----------------
    uint32_t  txCount_ = 0;
    uint32_t  rxCount_ = 0;
    uint32_t  parseErr_ = 0;

    // ---------------- NVS mirror ----------------
    static const uint8_t NVS_MAX = 24;
    struct NvsEntry { char key[20]; float val; };
    NvsEntry  nvsEntries_[NVS_MAX];
    uint8_t   nvsCount_ = 0;

    // ---------------- Hiệu chuẩn L ----------------
    bool      spinCalActive_ = false;
    float     spinCalYaw0_   = 0.0f;
    float     spinCalOdom0_  = 0.0f;
    float     spinCalTarget_ = 0.0f;   // rad
    uint8_t   spinCalTurns_  = 0;
    uint32_t  spinCalStartMs_ = 0;
};
