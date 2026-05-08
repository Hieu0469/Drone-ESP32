#pragma once
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "flight_controller.h"

#define BT_DEVICE_NAME    "ESP32_Drone"
#define BT_TIMEOUT_MS     500     // mất data 500ms → dừng

// Tốc độ tăng/giảm mỗi lần gọi update()
// update() gọi mỗi 20ms (xem JOYSTICK_UPDATE_MS)
#define THR_STEP          0.5f    // %   mỗi 20ms
#define ANGLE_STEP        0.3f    // độ  mỗi 20ms
#define YAW_STEP          3.0f    // deg/s mỗi 20ms

#define THR_MIN           0.0f
#define THR_MAX_DEFAULT       55.0f
#define INTEGRAL_LIMIT_DEFAULT 2.0f

#define ANGLE_MAX         30.0f
#define YAW_MAX           18.0f

#define JOYSTICK_UPDATE_MS  20   // xử lý ramping mỗi 20ms

// =============================================
// TRẠNG THÁI NÚT ĐANG GIỮ
// =============================================
enum class ActiveKey {
  NONE,
  THR_UP, THR_DOWN,
  ROLL_LEFT, ROLL_RIGHT,
  PITCH_FWD, PITCH_BACK,
  YAW_LEFT,  YAW_RIGHT
};

class BTController {
public:
  BTController(FlightController& fc);

  void begin();
  void update();   // gọi trong loop()

  bool isConnected()       const { return connected; }
  const RCInput& getRCInput() const { return rc; }

  void sendTelemetry(const SensorData& s,
                     const MotorOutput& m);

  float getThrMax()        const { return thrMax; }        // ← THÊM
  float getIntegralLimit() const { return integralLimit; } // ← THÊM
private:
  BluetoothSerial bt;
  FlightController& fc;

  RCInput   rc;
  ActiveKey activeKey;
  bool      connected;

  uint32_t  lastReceiveTime;
  uint32_t  lastRampTime;
  uint32_t  lastTelemetryTime;

  String    rxBuffer;

  float thrMax;           // ← THÊM
  float integralLimit;    // ← THÊM

  // Xử lý ký tự nhận được
  void handleChar(char c);

  // Ramping: tăng/giảm dần theo activeKey
  void applyRamp();

  // Gửi phản hồi về app
  void sendStatus();
  void sendPIDStatus();

  // PID command từ terminal (nhiều ký tự)
  void parsePIDCommand(const String& cmd);

  float clamp(float val, float mn, float mx);

  void parseConfigCommand(const String& cmd);  // ← THÊM

};