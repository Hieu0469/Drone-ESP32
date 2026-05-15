#include "flight_controller.h"
#include <Preferences.h>

// =============================================
// CONSTRUCTOR
// =============================================
FlightController::FlightController()
  : pidRoll (PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,
             PID_ROLL_LIMIT,  PID_INTEGRAL_LIMIT),
    pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
             PID_PITCH_LIMIT, PID_INTEGRAL_LIMIT),
    pidYaw  (PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,
             PID_YAW_LIMIT,   PID_INTEGRAL_LIMIT),
    armed(false)
{
  motorOut = {0, 0, 0, 0};
}

void FlightController::begin() {
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  loadTrim();
  armed = true;
  Serial.println("[FC] Flight Controller ready!");
}

// =============================================
// UPDATE — gọi mỗi 1ms
// =============================================
void FlightController::update(const RCInput& rc,
                               const SensorData& sensor) {
  if (!armed) {
    motorOut = {0, 0, 0, 0};
    return;
  }

  // Throttle quá thấp → reset integral, không bay
  if (rc.throttle < MIN_THROTTLE_ARM) {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    motorOut = {0, 0, 0, 0};
    return;
  }

  // --- PID Roll: setpoint=rc.roll, measured=sensor.roll ---
  float rollOut  = pidRoll.update(rc.roll,   sensor.roll,  FC_DT);

  // --- PID Pitch: setpoint=rc.pitch, measured=sensor.pitch ---
  float pitchOut = pidPitch.update(rc.pitch, sensor.pitch, FC_DT);

  // --- PID Yaw: setpoint=rc.yaw, measured=sensor.gz ---
  float yawOut   = pidYaw.update(rc.yaw,     sensor.gz,    FC_DT);

  // --- Mix vào motor ---
  mixMotors(rc.throttle, rollOut, pitchOut, yawOut);
}

// =============================================
// MOTOR MIXING
//
//      FRONT
//  FL(↺)  FR(↻)
//   BL(↻)  BR(↺)
//      BACK
//
// Roll+  → nghiêng phải → FL,BL tăng / FR,BR giảm
// Pitch+ → nghiêng tới  → BL,BR tăng / FL,FR giảm
// Yaw+   → xoay phải    → FL,BR tăng / FR,BL giảm
// =============================================
void FlightController::mixMotors(float throttle,
                                  float rollOut,
                                  float pitchOut,
                                  float yawOut) {
  float fl=0, fr=0, bl=0, br=0;
  fl = throttle + rollOut - pitchOut + yawOut;  // 25
  fr = throttle - rollOut - pitchOut - yawOut;   // 26
  bl = throttle + rollOut + pitchOut - yawOut;   //27
  br = throttle - rollOut + pitchOut + yawOut;   //14

  // Áp dụng trim — cộng thêm offset cho motor yếu
  // fl += motorTrim[0];
  // fr += motorTrim[1];
  // bl += motorTrim[2];
  // br += motorTrim[3];

  // Clamp 0 ~ MAX_THROTTLE
  motorOut.fl = clamp(fl, 0.0f, maxThrottle);
  motorOut.fr = clamp(fr, 0.0f, maxThrottle);
  motorOut.bl = clamp(bl, 0.0f, maxThrottle);
  motorOut.br = clamp(br, 0.0f, maxThrottle);
}

// =============================================
// DISARM
// =============================================
void FlightController::disarm() {
  armed    = false;
  motorOut = {0, 0, 0, 0};
  pidRoll.reset();
  pidPitch.reset();
  pidYaw.reset();
  Serial.println("[FC] Disarmed!");
}

// =============================================
// TUNING PID QUA SERIAL
// Cú pháp: "rp:1.5" "ri:0.05" "rd:0.8"
//           "pp:1.5" "pi:0.05" "pd:0.8"
//           "yp:3.0" "yi:0.02" "yd:0.0"
// =============================================
void FlightController::tunePID(const String& cmd) {
  if (cmd.length() < 4) return;

  String axis  = cmd.substring(0, 1);  // r/p/y
  String param = cmd.substring(1, 2);  // p/i/d
  float  val   = cmd.substring(3).toFloat();

  PIDController* pid = nullptr;
  if      (axis == "r") pid = &pidRoll;
  else if (axis == "p") pid = &pidPitch;
  else if (axis == "y") pid = &pidYaw;
  else return;

  float kp = pid->getKp();
  float ki = pid->getKi();
  float kd = pid->getKd();

  if      (param == "p") kp = val;
  else if (param == "i") ki = val;
  else if (param == "d") kd = val;

  pid->setGains(kp, ki, kd);
  printPIDStatus();
}

// =============================================
// PRINT PID STATUS
// =============================================
void FlightController::printPIDStatus() {
  Serial.printf(
    "[PID] Roll  Kp:%.3f Ki:%.3f Kd:%.3f\n"
    "[PID] Pitch Kp:%.3f Ki:%.3f Kd:%.3f\n"
    "[PID] Yaw   Kp:%.3f Ki:%.3f Kd:%.3f\n",
    pidRoll.getKp(),  pidRoll.getKi(),  pidRoll.getKd(),
    pidPitch.getKp(), pidPitch.getKi(), pidPitch.getKd(),
    pidYaw.getKp(),   pidYaw.getKi(),   pidYaw.getKd()
  );
}

float FlightController::clamp(float val, float mn, float mx) {
  return val < mn ? mn : (val > mx ? mx : val);
}

void FlightController::setMotorTrim(int motorIdx, float trim) {
  if (motorIdx < 0 || motorIdx >= 4) return;
  motorTrim[motorIdx] = clamp(trim, TRIM_MIN, TRIM_MAX);
  Serial.printf("[TRIM] Motor %d = %.1f%%\n",
                motorIdx, motorTrim[motorIdx]);
}

float FlightController::getMotorTrim(int motorIdx) const {
  if (motorIdx < 0 || motorIdx >= 4) return 0;
  return motorTrim[motorIdx];
}

void FlightController::printTrimStatus() {
  Serial.printf(
    "[TRIM] FL:%.1f  FR:%.1f  BL:%.1f  BR:%.1f\n",
    motorTrim[0], motorTrim[1],
    motorTrim[2], motorTrim[3]
  );
}

void FlightController::saveTrim() {
  Preferences prefs;
  prefs.begin("drone", false);
  prefs.putFloat("trim_fl", motorTrim[0]);
  prefs.putFloat("trim_fr", motorTrim[1]);
  prefs.putFloat("trim_bl", motorTrim[2]);
  prefs.putFloat("trim_br", motorTrim[3]);
  prefs.end();
  Serial.println("[TRIM] Saved to NVS");
}

void FlightController::loadTrim() {
  Preferences prefs;
  prefs.begin("drone", true);  // read-only
  motorTrim[0] = prefs.getFloat("trim_fl", 0.0f);
  motorTrim[1] = prefs.getFloat("trim_fr", 0.0f);
  motorTrim[2] = prefs.getFloat("trim_bl", 0.0f);
  motorTrim[3] = prefs.getFloat("trim_br", 0.0f);
  prefs.end();
  Serial.println("[TRIM] Loaded from NVS");
  printTrimStatus();
}