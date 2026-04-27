#pragma once
#include <Arduino.h>

// =============================================
// CẤU HÌNH PID MẶC ĐỊNH
// =============================================

// PID Góc (Angle mode)
#define PID_ROLL_KP     0.0f
#define PID_ROLL_KI     0.0f
#define PID_ROLL_KD     0.0f

#define PID_PITCH_KP    0.0f
#define PID_PITCH_KI    0.0f
#define PID_PITCH_KD    0.0f

#define PID_YAW_KP      0.0f
#define PID_YAW_KI      0.0f
#define PID_YAW_KD      0.0f

// Giới hạn output PID
#define PID_ROLL_LIMIT   300.0f
#define PID_PITCH_LIMIT  300.0f
#define PID_YAW_LIMIT    200.0f

// Giới hạn integral (anti-windup)
#define PID_INTEGRAL_LIMIT  100.0f

struct PIDConfig {
  float kp, ki, kd;
  float outputLimit;
  float integralLimit;
};

// =============================================
// CLASS PID
// =============================================
class PIDController {
public:
  PIDController(float kp, float ki, float kd,
                float outputLimit    = 300.0f,
                float integralLimit  = 100.0f);

  // Cập nhật PID, trả về output
  // setpoint: giá trị mong muốn
  // measured: giá trị đo được
  // dt: thời gian (giây)
  float update(float setpoint, float measured, float dt);

  // Reset integral & derivative
  void reset();

  // Tuning runtime
  void setGains(float kp, float ki, float kd);
  void setOutputLimit(float limit);
  void setIntegralLimit(float limit);

  // Getters
  float getKp() const { return kp; }
  float getKi() const { return ki; }
  float getKd() const { return kd; }
  float getError()      const { return lastError; }
  float getIntegral()   const { return integral; }
  float getDerivative() const { return derivative; }
  float getOutput()     const { return output; }

private:
  float kp, ki, kd;
  float outputLimit;
  float integralLimit;

  float lastError;
  float integral;
  float derivative;
  float output;

  bool  firstRun;

  float clamp(float val, float limit);
};