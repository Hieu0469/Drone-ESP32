#include <Arduino.h>
#include "esc_controller.h"
#include "mpu6050_handler.h"
#include "flight_controller.h"
#include "bt_controller.h"

// =============================================
// OBJECTS
// =============================================
ESCController    esc;
MPU6050Handler   imu;
FlightController fc;
BTController     bt(fc);

// =============================================
// TIMER 1ms
// =============================================
hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool tickReady = false;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  tickReady = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// =============================================
// SETUP
// =============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Drone + BT Control ===");

  // IMU
  if (!imu.begin()) {
    Serial.println("[ERROR] MPU6050 không tìm thấy!");
    while (1);
  }

  // ESC
  esc.begin();
  esc.armAll();

  // Flight Controller
  fc.begin();

  // Bluetooth
  bt.begin();

  // Timer 1ms
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);

  Serial.println("Sẵn sàng! Kết nối Bluetooth: ESP32_Drone");
}

// =============================================
// LOOP
// =============================================
void loop() {
  // --- Bluetooth: nhận lệnh từ app ---
  bt.update();

  // --- Vòng lặp PID 1ms ---
  if (tickReady) {
    portENTER_CRITICAL(&timerMux);
    tickReady = false;
    portEXIT_CRITICAL(&timerMux);

    // 1. Đọc IMU
    imu.update();
    const SensorData& s = imu.getData();

    // 2. Lấy RC input từ Bluetooth
    const RCInput& rc = bt.getRCInput();

    // 3. Tính PID
    fc.update(rc, s);

    // 4. Gửi ESC
    const MotorOutput& m = fc.getMotorOutput();
    esc.setAllThrottle(m.fl, m.fr, m.bl, m.br);

    // 5. Gửi telemetry về app
    bt.sendTelemetry(s, m);
  }
}