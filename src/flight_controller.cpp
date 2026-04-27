#include "flight_controller.h"

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
  float fl = throttle + rollOut - pitchOut + yawOut;
  float fr = throttle - rollOut - pitchOut - yawOut;
  float bl = throttle + rollOut + pitchOut - yawOut;
  float br = throttle - rollOut + pitchOut + yawOut;

  // Clamp 0 ~ MAX_THROTTLE
  motorOut.fl = clamp(fl, 0.0f, MAX_THROTTLE);
  motorOut.fr = clamp(fr, 0.0f, MAX_THROTTLE);
  motorOut.bl = clamp(bl, 0.0f, MAX_THROTTLE);
  motorOut.br = clamp(br, 0.0f, MAX_THROTTLE);
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