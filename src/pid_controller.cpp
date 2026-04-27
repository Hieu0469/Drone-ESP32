#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float outputLimit,
                             float integralLimit)
  : kp(kp), ki(ki), kd(kd),
    outputLimit(outputLimit),
    integralLimit(integralLimit),
    lastError(0), integral(0),
    derivative(0), output(0),
    firstRun(true)
{}

// =============================================
// UPDATE
// =============================================
float PIDController::update(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) return output;

  float error = setpoint - measured;

  // --- Proportional ---
  float P = kp * error;

  // --- Integral (anti-windup clamp) ---
  integral += error * dt;
  integral  = clamp(integral, integralLimit);
  float I   = ki * integral;

  // --- Derivative (trên measured, không phải error
  //     tránh derivative kick khi setpoint thay đổi đột ngột)
  if (firstRun) {
    derivative = 0;
    firstRun   = false;
  } else {
    derivative = -(measured - lastError) / dt;
    // Lưu ý: lastError ở đây lưu measured trước
  }
  float D = kd * derivative;

  lastError = measured;  // lưu measured (không phải error)

  // --- Tổng output ---
  output = clamp(P + I + D, outputLimit);
  return output;
}

// =============================================
// RESET
// =============================================
void PIDController::reset() {
  lastError  = 0;
  integral   = 0;
  derivative = 0;
  output     = 0;
  firstRun   = true;
}

void PIDController::setGains(float p, float i, float d) {
  kp = p; ki = i; kd = d;
}

void PIDController::setOutputLimit(float limit) {
  outputLimit = limit;
}

void PIDController::setIntegralLimit(float limit) {
  integralLimit = limit;
}

float PIDController::clamp(float val, float limit) {
  if (val >  limit) return  limit;
  if (val < -limit) return -limit;
  return val;
}