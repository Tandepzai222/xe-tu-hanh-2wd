/**
 * main.cpp — khởi tạo hệ thống và tạo 2 task RTOS.
 *
 * KIẾN TRÚC TASK (xem docs/04 §2 — đây là quyết định thiết kế quan trọng nhất):
 *
 *   Core 0 : WiFi / TCP-IP stack (hệ thống quản lý) — có thể block tuỳ ý
 *
 *   Core 1 : controlTask  priority 10, chu kỳ 10 ms   ← ƯU TIÊN CAO
 *            loopTask     priority  1, comms + telemetry
 *
 * Nhờ controlTask có priority cao hơn, nó LUÔN được chạy đúng hạn mỗi 10 ms
 * bất kể loopTask/WiFi đang làm gì → Ts ổn định → PID mới tune được.
 *
 * ⚠️ dt truyền vào robot.step() là dt ĐO ĐƯỢC (micros), không phải 0.010 giả định.
 *    Nhờ vậy PID vẫn đúng ngay cả khi có jitter.
 */
#include <Arduino.h>
#include "config.h"
#include "robot.h"
#include "comms.h"

// ============================================================================
//  Đối tượng toàn cục
// ============================================================================
Robot g_robot;
Comms g_comms;

static TaskHandle_t g_ctrlTask = nullptr;

// ============================================================================
//  Task điều khiển — chạy trên core 1, priority cao
// ============================================================================
static void controlTask(void* pv) {
    (void)pv;

    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_US / 1000u);
    TickType_t lastWake     = xTaskGetTickCount();
    uint32_t   prevUs       = micros();

    for (;;) {
        const uint32_t nowUs = micros();
        const float dt = (float)(uint32_t)(nowUs - prevUs) * 1e-6f;
        prevUs = nowUs;

        g_robot.step(dt);

        vTaskDelayUntil(&lastWake, period);
    }
}

// ============================================================================
//  setup()
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(400);

    Serial.println();
    Serial.println(F("=========================================================="));
    Serial.printf ( "   XE TU HANH 2 BANH VI SAI - ESP32        FW %s\n", FW_VERSION);
    Serial.println(F("=========================================================="));

    // ------------------------------------------------------------------
    // 1. Khởi tạo phần cứng. Robot::begin() sẽ:
    //      - tắt TB6612 (STBY=LOW) TRƯỚC TIÊN để motor không giật lúc boot
    //      - cấu hình PCNT cho 2 encoder
    //      - cấu hình LEDC cho 2 kênh PWM
    //      - cấu hình PID
    //      - khởi tạo IMU
    //      - bật lại TB6612
    // ------------------------------------------------------------------
    g_robot.begin();

    // ------------------------------------------------------------------
    // 2. Hiệu chuẩn bias gyro — BẮT BUỘC mỗi lần khởi động.
    //    Robot phải đứng yên hoàn toàn trong ~2 giây.
    // ------------------------------------------------------------------
    Serial.println(F("[BOOT] Hieu chuan IMU. KHONG CHAM VAO ROBOT trong 2 giay..."));
    delay(1500);
    if (g_robot.cmdCalImu()) {
        Serial.println(F("[BOOT] Hieu chuan IMU thanh cong."));
    } else {
        Serial.println(F("[BOOT] CANH BAO: hieu chuan IMU that bai. "
                         "Che do HEADING/GOTO se khong dung duoc."));
    }

    // ------------------------------------------------------------------
    // 3. WiFi + console
    // ------------------------------------------------------------------
    g_comms.begin(&g_robot);

    // ------------------------------------------------------------------
    // 4. Tạo task điều khiển
    // ------------------------------------------------------------------
    const BaseType_t ok = xTaskCreatePinnedToCore(
        controlTask, "control",
        CONTROL_TASK_STACK, nullptr,
        CONTROL_TASK_PRIO, &g_ctrlTask,
        CONTROL_TASK_CORE);

    if (ok != pdPASS) {
        Serial.println(F("[BOOT] LOI NGHIEM TRONG: khong tao duoc controlTask!"));
        Serial.println(F("[BOOT] Robot se KHONG dieu khien duoc. Kiem tra CONTROL_TASK_STACK."));
    } else {
        Serial.printf("[BOOT] controlTask: core %d, priority %d, chu ky %d ms\n",
                      CONTROL_TASK_CORE, CONTROL_TASK_PRIO, CONTROL_PERIOD_US / 1000);
    }

    Serial.printf("[BOOT] Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.printf("[BOOT] d_pc = %.6f mm/xung   (%.1f xung/m)\n",
                  DISTANCE_PER_COUNT_M * 1000.0f, COUNTS_PER_METER);
    Serial.println(F("[BOOT] San sang. Go `help` de xem danh sach lenh."));
}

// ============================================================================
//  loop() — chạy ở loopTask (core 1, priority 1)
// ============================================================================
void loop() {
    g_comms.loop();
    delay(1);          // nhường CPU; KHÔNG ảnh hưởng controlTask (priority 10)
}
