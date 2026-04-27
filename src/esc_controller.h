#pragma once
#include <Arduino.h>

// =============================================
// CẤU HÌNH PIN & PWM
// =============================================
#define MOTOR_FL_PIN   25   // Front Left
#define MOTOR_FR_PIN   26   // Front Right
#define MOTOR_BL_PIN   27   // Back Left
#define MOTOR_BR_PIN   14   // Back Right

#define PWM_FREQ       50        // 50Hz chuẩn ESC
#define PWM_RESOLUTION 16        // 16-bit → 0~65535
#define PWM_MIN_US     1000      // 1000us = throttle min / arm
#define PWM_MAX_US     2000      // 2000us = throttle max
#define PWM_ARM_US     1000      // xung arm ESC
#define PWM_STOP_US    900       // dưới min → dừng hẳn

// Throttle dạng % (0.0 ~ 100.0)
#define THROTTLE_MIN   0.0f
#define THROTTLE_MAX   100.0f

// =============================================
// MOTOR INDEX
// =============================================
enum MotorID {
  MOTOR_FL = 0,
  MOTOR_FR = 1,
  MOTOR_BL = 2,
  MOTOR_BR = 3,
  MOTOR_COUNT = 4
};

// =============================================
// CLASS ESC CONTROLLER
// =============================================
class ESCController {
public:
  ESCController();

  // Khởi tạo PWM channels
  void begin();

  // Arm tất cả ESC (bắt buộc trước khi bay)
  void armAll();

  // Dừng khẩn cấp tất cả motor
  void emergencyStop();

  // Đặt throttle 1 motor (0.0 ~ 100.0 %)
  void setThrottle(MotorID motor, float throttle);

  // Đặt throttle 4 motor cùng lúc
  void setAllThrottle(float fl, float fr, float bl, float br);

  // Đặt throttle bằng microsecond trực tiếp (1000~2000)
  void setMicroseconds(MotorID motor, uint16_t us);

  // Lấy throttle hiện tại (%)
  float getThrottle(MotorID motor) const;

  // Lấy microsecond hiện tại
  uint16_t getMicroseconds(MotorID motor) const;

  // Kiểm tra ESC đã arm chưa
  bool isArmed() const { return armed; }

private:
  // PWM channel tương ứng mỗi motor
  static const uint8_t pwmChannels[MOTOR_COUNT];
  static const uint8_t motorPins[MOTOR_COUNT];

  float    currentThrottle[MOTOR_COUNT];
  uint16_t currentUs[MOTOR_COUNT];
  bool     armed;

  // Chuyển microsecond → duty cycle 16-bit
  uint32_t usToDuty(uint16_t us);

  // Clamp giá trị trong khoảng
  float clamp(float val, float minVal, float maxVal);
  uint16_t clampUs(uint16_t us);
};