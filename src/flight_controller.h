#pragma once
#include <Arduino.h>
#include "pid_controller.h"
#include "esc_controller.h"
#include "mpu6050_handler.h"

// =============================================
// CẤU HÌNH BAY
// =============================================
#define FC_DT              0.001f   // 1ms loop

// Giới hạn góc nghiêng an toàn (độ)
#define MAX_ANGLE_ROLL     30.0f
#define MAX_ANGLE_PITCH    30.0f

// Throttle tối thiểu để PID hoạt động
#define MIN_THROTTLE_ARM   10.0f
// Throttle tối đa cho phép
#define MAX_THROTTLE       85.0f

// =============================================
// INPUT TỪ REMOTE CONTROL
// =============================================
struct RCInput {
  float throttle;    // 0.0 ~ 100.0 %
  float roll;        // -MAX_ANGLE_ROLL  ~ +MAX_ANGLE_ROLL  (độ)
  float pitch;       // -MAX_ANGLE_PITCH ~ +MAX_ANGLE_PITCH (độ)
  float yaw;         // -180 ~ +180 (deg/s)
};

// =============================================
// OUTPUT ĐI VÀO ESC
// =============================================
struct MotorOutput {
  float fl, fr, bl, br;  // 0.0 ~ 100.0 %
};

// =============================================
// CLASS FLIGHT CONTROLLER
// =============================================
class FlightController {
public:
  FlightController();

  void begin();
  void update(const RCInput& rc, const SensorData& sensor);

  // Emergency stop
  void disarm();

  // Lấy output motor
  const MotorOutput& getMotorOutput() const { return motorOut; }

  // Tuning PID runtime qua Serial
  void tunePID(const String& cmd);

  // In thông số PID hiện tại
  void printPIDStatus();

  // Trạng thái
  bool isArmed() const { return armed; }

  PIDController& getRollPID()  { return pidRoll;  }
  PIDController& getPitchPID() { return pidPitch; }
  PIDController& getYawPID()   { return pidYaw;   }

private:
  // 3 PID controllers
  PIDController pidRoll;
  PIDController pidPitch;
  PIDController pidYaw;

  MotorOutput motorOut;
  bool armed;

  // Mix PID output → 4 motor
  void mixMotors(float throttle,
                 float rollOut,
                 float pitchOut,
                 float yawOut);

  float clamp(float val, float mn, float mx);
};