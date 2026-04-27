#include "esc_controller.h"

// =============================================
// STATIC MEMBERS
// =============================================
const uint8_t ESCController::motorPins[MOTOR_COUNT] = {
  MOTOR_FL_PIN,  // FL
  MOTOR_FR_PIN,  // FR
  MOTOR_BL_PIN,  // BL
  MOTOR_BR_PIN   // BR
};

const uint8_t ESCController::pwmChannels[MOTOR_COUNT] = {
  0, 1, 2, 3  // LEDC channel 0~3
};

// =============================================
// CONSTRUCTOR
// =============================================
ESCController::ESCController() : armed(false) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    currentThrottle[i] = 0.0f;
    currentUs[i]       = PWM_STOP_US;
  }
}

// =============================================
// BEGIN — Khởi tạo LEDC PWM
// =============================================
void ESCController::begin() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    ledcSetup(pwmChannels[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(motorPins[i], pwmChannels[i]);

    // Gửi xung STOP trước
    ledcWrite(pwmChannels[i], usToDuty(PWM_STOP_US));
  }

  Serial.println("[ESC] PWM channels initialized");
  Serial.printf("  FL: GPIO%d  FR: GPIO%d  BL: GPIO%d  BR: GPIO%d\n",
                MOTOR_FL_PIN, MOTOR_FR_PIN,
                MOTOR_BL_PIN, MOTOR_BR_PIN);
}

// =============================================
// ARM — Gửi xung 1000us để arm ESC
// =============================================
void ESCController::armAll() {
  Serial.println("[ESC] Arming... (2 giây)");

  for (int i = 0; i < MOTOR_COUNT; i++) {
    ledcWrite(pwmChannels[i], usToDuty(PWM_ARM_US));
    currentUs[i]       = PWM_ARM_US;
    currentThrottle[i] = 0.0f;
  }

  delay(2000);  // ESC cần 2s để nhận tín hiệu arm
  armed = true;
  Serial.println("[ESC] Armed! San sang bay.");
}

// =============================================
// EMERGENCY STOP
// =============================================
void ESCController::emergencyStop() {
  armed = false;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    ledcWrite(pwmChannels[i], usToDuty(PWM_STOP_US));
    currentUs[i]       = PWM_STOP_US;
    currentThrottle[i] = 0.0f;
  }
  Serial.println("[ESC] !!! EMERGENCY STOP !!!");
}

// =============================================
// SET THROTTLE (%) cho 1 motor
// =============================================
void ESCController::setThrottle(MotorID motor, float throttle) {
  if (!armed) {
    Serial.println("[ESC] Chua arm! Goi armAll() truoc.");
    return;
  }

  throttle = clamp(throttle, THROTTLE_MIN, THROTTLE_MAX);

  // Map 0~100% → 1000~2000us
  uint16_t us = (uint16_t)(PWM_MIN_US
              + (throttle / 100.0f) * (PWM_MAX_US - PWM_MIN_US));

  currentThrottle[motor] = throttle;
  currentUs[motor]       = us;

  ledcWrite(pwmChannels[motor], usToDuty(us));
}

// =============================================
// SET THROTTLE cho 4 motor cùng lúc
// =============================================
void ESCController::setAllThrottle(float fl, float fr,
                                    float bl, float br) {
  setThrottle(MOTOR_FL, fl);
  setThrottle(MOTOR_FR, fr);
  setThrottle(MOTOR_BL, bl);
  setThrottle(MOTOR_BR, br);
}

// =============================================
// SET MICROSECONDS trực tiếp
// =============================================
void ESCController::setMicroseconds(MotorID motor, uint16_t us) {
  if (!armed) return;

  us = clampUs(us);
  currentUs[motor] = us;
  currentThrottle[motor] = (float)(us - PWM_MIN_US)
                          / (PWM_MAX_US - PWM_MIN_US) * 100.0f;

  ledcWrite(pwmChannels[motor], usToDuty(us));
}

// =============================================
// GETTERS
// =============================================
float ESCController::getThrottle(MotorID motor) const {
  return currentThrottle[motor];
}

uint16_t ESCController::getMicroseconds(MotorID motor) const {
  return currentUs[motor];
}

// =============================================
// PRIVATE: Chuyển us → duty 16-bit
// 50Hz → chu kỳ 20ms = 20000us
// duty = (us / 20000) * 65535
// =============================================
uint32_t ESCController::usToDuty(uint16_t us) {
  return (uint32_t)((float)us / 20000.0f * 65535.0f);
}

float ESCController::clamp(float val, float mn, float mx) {
  return val < mn ? mn : (val > mx ? mx : val);
}

uint16_t ESCController::clampUs(uint16_t us) {
  return us < PWM_MIN_US ? PWM_MIN_US
       : us > PWM_MAX_US ? PWM_MAX_US : us;
}