#include "bt_controller.h"

BTController::BTController(FlightController& fc)
  : fc(fc),
    activeKey(ActiveKey::NONE),
    connected(false),
    lastReceiveTime(0),
    lastRampTime(0),
    lastTelemetryTime(0),
    thrMax(THR_MAX_DEFAULT),           // ← THÊM
    integralLimit(INTEGRAL_LIMIT_DEFAULT) // ← THÊM
{
  rc = {0.0f, 0.0f, 0.0f, 0.0f};
}

// =============================================
// BEGIN
// =============================================
void BTController::begin() {
  if (!bt.begin(BT_DEVICE_NAME)) {
    Serial.println("[BT] Khởi động thất bại!");
    return;
  }
  Serial.printf("[BT] Tên: %s — Đang chờ kết nối...\n",
                BT_DEVICE_NAME);
}

// =============================================
// UPDATE — gọi liên tục trong loop()
// =============================================
void BTController::update() {
  bool nowConnected = bt.connected();

  // --- Xử lý kết nối / ngắt ---
  if (nowConnected && !connected) {
    connected       = true;
    lastReceiveTime = millis();
    Serial.println("[BT] Đã kết nối!");
    bt.println("CONNECTED:ESP32_Drone");
    sendStatus();
  }

  if (!nowConnected && connected) {
    connected = false;
    activeKey = ActiveKey::NONE;
    rc        = {0, 0, 0, 0};
    Serial.println("[BT] Mất kết nối! Reset RC.");
  }

  if (!connected) return;

  // --- Đọc từng byte từ app ---
  while (bt.available()) {
    char c = (char)bt.read();
    lastReceiveTime = millis();

    // Ký tự đơn → xử lý ngay
    if (c == '\n' || c == '\r') {
      if (rxBuffer.length() > 0) {
        parsePIDCommand(rxBuffer);
        rxBuffer = "";
      }
      continue;
    }

    // Ký tự lệnh đơn (in hoa)
    // Nếu chỉ 1 ký tự và không phải số/dấu chấm
    // → joystick command
    if (rxBuffer.length() == 0 && isAlpha(c) && isUpperCase(c)) {
      handleChar(c);
    } else {
      // Tích lũy thành chuỗi PID command
      rxBuffer += c;
      if (rxBuffer.length() > 64) rxBuffer = "";
    }
  }

  // --- Timeout mất data ---
//   if (millis() - lastReceiveTime > BT_TIMEOUT_MS) {
//     if (rc.throttle > 0) {
//       rc        = {0, 0, 0, 0};
//       activeKey = ActiveKey::NONE;
//       Serial.println("[BT] Timeout! Reset RC.");
//     }
//   }

  // --- Ramping mỗi 20ms ---
  if (millis() - lastRampTime >= JOYSTICK_UPDATE_MS) {
    lastRampTime = millis();
    applyRamp();
  }
}

// =============================================
// HANDLE CHAR — xử lý ký tự đơn từ joystick
// =============================================
void BTController::handleChar(char c) {
  Serial.printf("[BT] Key: '%c'\n", c);

  switch (c) {

    // --- THROTTLE ---
    case 'U':  activeKey = ActiveKey::THR_UP;    break;
    case 'D':  activeKey = ActiveKey::THR_DOWN;  break;

    // --- ROLL ---
    case 'L':  activeKey = ActiveKey::ROLL_LEFT;  break;
    case 'R':  activeKey = ActiveKey::ROLL_RIGHT; break;

    // --- PITCH ---
    case 'F':  activeKey = ActiveKey::PITCH_FWD;  break;
    case 'B':  activeKey = ActiveKey::PITCH_BACK; break;

    // --- YAW ---
    case 'Q':  activeKey = ActiveKey::YAW_LEFT;  break;
    case 'E':  activeKey = ActiveKey::YAW_RIGHT; break;

    // --- HOLD: thả nút → giữ nguyên giá trị ---
    case 'H':
      activeKey = ActiveKey::NONE;
      // Roll, Pitch, Yaw về 0 khi thả
      // Throttle giữ nguyên
      if (rc.roll > 0) rc.roll = clamp(rc.roll - ANGLE_STEP, 0.0f, ANGLE_MAX);
      else if (rc.roll < 0) rc.roll = clamp(rc.roll + ANGLE_STEP, -ANGLE_MAX, 0.0f);
      
      if(rc.pitch > 0) rc.pitch = clamp(rc.pitch - ANGLE_STEP, 0.0f, ANGLE_MAX);
      else if (rc.pitch < 0) rc.pitch = clamp(rc.pitch + ANGLE_STEP, -ANGLE_MAX, 0.0f);
      
      if(rc.yaw > 0) rc.yaw = clamp(rc.yaw - YAW_STEP, 0.0f, YAW_MAX);
      else if (rc.yaw < 0) rc.yaw = clamp(rc.yaw + YAW_STEP, -YAW_MAX, 0.0f);
      
      Serial.printf("[BT] HOLD — Thr:%.1f%%\n", rc.throttle);
      break;

    // --- EMERGENCY STOP ---
    case 'X':
      activeKey   = ActiveKey::NONE;
      rc          = {0.0f, 0.0f, 0.0f, 0.0f};
      fc.disarm();
      Serial.println("[BT] !!! EMERGENCY STOP !!!");
      bt.println("STOP:OK");
      break;

    // --- ARM ---
    case 'A':
      fc.begin();
      Serial.println("[BT] Armed!");
      bt.println("ARM:OK");
      break;

    // --- DISARM ---
    case 'S':
      activeKey = ActiveKey::NONE;
      rc        = {0.0f, 0.0f, 0.0f, 0.0f};
      fc.disarm();
      bt.println("DISARM:OK");
      break;

    // --- STATUS ---
    case 'P':
      sendStatus();
      sendPIDStatus();
      break;

    default:
      Serial.printf("[BT] Unknown key: '%c'\n", c);
      break;
  }
}

// =============================================
// APPLY RAMP — tăng/giảm dần mỗi 20ms
// =============================================
void BTController::applyRamp() {
  if(rc.roll > 45.0f || rc.roll < -45.0f || rc.pitch > 45.0f || rc.pitch < -45.0f) {
        activeKey = ActiveKey::NONE;
        rc        = {0.0f, 0.0f, 0.0f, 0.0f};
        fc.disarm();
  }
  switch (activeKey) {

    case ActiveKey::THR_UP:
      rc.throttle = clamp(rc.throttle + THR_STEP,
                          THR_MIN, thrMax);

      break;

    case ActiveKey::THR_DOWN:
      rc.throttle = clamp(rc.throttle - THR_STEP,
                          THR_MIN, thrMax);
      break;

    case ActiveKey::ROLL_LEFT:
      rc.roll = clamp(rc.roll - ANGLE_STEP,
                      -ANGLE_MAX, ANGLE_MAX);
      break;

    case ActiveKey::ROLL_RIGHT:
      rc.roll = clamp(rc.roll + ANGLE_STEP,
                      -ANGLE_MAX, ANGLE_MAX);
      break;

    case ActiveKey::PITCH_FWD:
      rc.pitch = clamp(rc.pitch - ANGLE_STEP,
                       -ANGLE_MAX, ANGLE_MAX);
      break;

    case ActiveKey::PITCH_BACK:
      rc.pitch = clamp(rc.pitch + ANGLE_STEP,
                       -ANGLE_MAX, ANGLE_MAX);
      break;

    case ActiveKey::YAW_LEFT:
      rc.yaw = clamp(rc.yaw - YAW_STEP,
                     -YAW_MAX, YAW_MAX);
      break;

    case ActiveKey::YAW_RIGHT:
      rc.yaw = clamp(rc.yaw + YAW_STEP,
                     -YAW_MAX, YAW_MAX);
      break;

    case ActiveKey::NONE:
    default:
      break;
  }
}

// =============================================
// PARSE PID COMMAND (chuỗi nhiều ký tự)
// rp:1.5 | ri:0.05 | rd:0.8
// pp:1.5 | pi:0.05 | pd:0.8
// yp:3.0 | yi:0.02 | yd:0.0
// pid    | arm     | stop
// =============================================
void BTController::parsePIDCommand(const String& cmd) {
  Serial.printf("[BT CMD] '%s'\n", cmd.c_str());

  if (cmd == "pid") {
    sendPIDStatus();

  } else if (cmd == "stop" || cmd == "x") {
    handleChar('X');

  } else if (cmd == "arm") {
    handleChar('A');

  } else if (cmd == "disarm") {
    handleChar('S');

  } 
  else if (cmd.startsWith("cfg:")) {
    parseConfigCommand(cmd.substring(4));
  }
  else if (cmd.length() >= 4 && cmd.charAt(2) == ':') {
    // PID tuning: "rp:1.5"
    fc.tunePID(cmd);
    bt.println("PID:UPDATED");
    sendPIDStatus();

  }
  else if (cmd.startsWith("trim:")) {
    parseTrimCommand(cmd.substring(5));
  }
  else if (cmd == "trim") {
    // Xem hướng dẫn
    bt.println("TRIM:fl:+5.0  fr:0  bl:+3.0  br:0");
    bt.println("TRIM:range -20.0 to +20.0");
    bt.println("TRIM:reset = ve 0 het");
    sendTrimStatus();
  }
   else {
    bt.println("ERR:UNKNOWN_CMD");
  }
}
// =============================================
// PARSE CONFIG COMMAND
// Cú pháp: "cfg:tmax:55.0"   → đặt THR_MAX
//           "cfg:ilim:2.0"   → đặt PID_INTEGRAL_LIMIT
//           "cfg:?"          → xem giá trị hiện tại
// =============================================
void BTController::parseConfigCommand(const String& cmd) {
  Serial.printf("[BT CFG] '%s'\n", cmd.c_str());

  // Xem giá trị hiện tại
  if (cmd == "?") {
    char buf[80];
    snprintf(buf, sizeof(buf),
      "CFG:tmax=%.1f,ilim=%.3f",
      thrMax, integralLimit
    );
    bt.println(buf);
    Serial.println(buf);
    return;
  }

  // Tách key:value
  int sep = cmd.indexOf(':');
  if (sep < 0) {
    bt.println("ERR:CFG_FORMAT");  // phải có dấu ':'
    return;
  }

  String key = cmd.substring(0, sep);
  float  val = cmd.substring(sep + 1).toFloat();

  if (key == "tmax") {
    // Giới hạn an toàn: 20% ~ 90%
    if (val < 20.0f || val > 90.0f) {
      bt.println("ERR:TMAX_RANGE(20-90)");
      return;
    }
    thrMax = val;

    // Áp dụng ngay vào RC
    // Nếu throttle hiện tại > thrMax mới → kéo xuống
    if (rc.throttle > thrMax) {
      rc.throttle = thrMax;
    }

    char buf[40];
    snprintf(buf, sizeof(buf), "CFG:tmax=%.1f OK", thrMax);
    bt.println(buf);
    Serial.println(buf);

  } else if (key == "ilim") {
    // Giới hạn an toàn: 0.1 ~ 50.0
    if (val < 0.1f || val > 50.0f) {
      bt.println("ERR:ILIM_RANGE(0.1-50)");
      return;
    }
    integralLimit = val;

    // Áp dụng ngay vào 3 PID controller
    fc.getRollPID().setIntegralLimit(integralLimit);
    fc.getPitchPID().setIntegralLimit(integralLimit);
    fc.getYawPID().setIntegralLimit(integralLimit);

    char buf[40];
    snprintf(buf, sizeof(buf), "CFG:ilim=%.3f OK", integralLimit);
    bt.println(buf);
    Serial.println(buf);

  }
  
  else if (cmd.startsWith("trim:")) {
    parseTrimCommand(cmd.substring(5));

  } else if (cmd == "trim") {
    // Xem hướng dẫn
    bt.println("TRIM:fl:+5.0  fr:0  bl:+3.0  br:0");
    bt.println("TRIM:range -20.0 to +20.0");
    bt.println("TRIM:reset = ve 0 het");
    sendTrimStatus();
  }
  
  else {
    bt.println("ERR:CFG_KEY_UNKNOWN");
  }
}
// =============================================
// GỬI TELEMETRY về app mỗi 100ms
// =============================================
void BTController::sendTelemetry(const SensorData& s,
                                   const MotorOutput& m) {
  if (!connected) return;
  if (millis() - lastTelemetryTime < 100) return;
  lastTelemetryTime = millis();

  char buf[180];
  snprintf(buf, sizeof(buf),
    "T:%.2f,%.2f,%.2f|M:%.1f,%.1f,%.1f,%.1f|R:%.1f,%.1f,%.1f,%.1f",
    s.pitch, s.roll, s.gz,
    m.fl, m.fr, m.bl, m.br,
    rc.throttle, rc.roll, rc.pitch, rc.yaw
  );
  bt.println(buf);
}

// =============================================
// STATUS & PID
// =============================================
void BTController::sendStatus() {
  char buf[160];
  snprintf(buf, sizeof(buf),
    "STATUS:armed=%d,thr=%.1f"
    "|TRIM:fl=%.1f,fr=%.1f,bl=%.1f,br=%.1f",
    fc.isArmed(), rc.throttle,
    fc.getMotorTrim(0), fc.getMotorTrim(1),
    fc.getMotorTrim(2), fc.getMotorTrim(3)
  );
  bt.println(buf);
}


void BTController::sendPIDStatus() {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "PID:"
    "rp=%.3f,ri=%.3f,rd=%.3f|"
    "pp=%.3f,pi=%.3f,pd=%.3f|"
    "yp=%.3f,yi=%.3f,yd=%.3f",
    fc.getRollPID().getKp(),
    fc.getRollPID().getKi(),
    fc.getRollPID().getKd(),
    fc.getPitchPID().getKp(),
    fc.getPitchPID().getKi(),
    fc.getPitchPID().getKd(),
    fc.getYawPID().getKp(),
    fc.getYawPID().getKi(),
    fc.getYawPID().getKd()
  );
  bt.println(buf);
}

float BTController::clamp(float val, float mn, float mx) {
  return val < mn ? mn : (val > mx ? mx : val);
}

void BTController::parseTrimCommand(const String& cmd) {
  Serial.printf("[BT TRIM] '%s'\n", cmd.c_str());

  // Xem giá trị hiện tại
  if (cmd == "?") {
    sendTrimStatus();
    return;
  }

  // Reset tất cả về 0
  if (cmd == "reset") {
    for (int i = 0; i < 4; i++)
      fc.setMotorTrim(i, 0.0f);
    bt.println("TRIM:reset OK");
    sendTrimStatus();
    return;
  }

  if (cmd == "save") {
    fc.saveTrim();
    bt.println("TRIM:saved to NVS");
    return;
  }

  // Tách motor:value — ví dụ "fl:+5.0"
  int sep = cmd.indexOf(':');
  if (sep < 0) {
    bt.println("ERR:TRIM_FORMAT(vi_du:trim:fl:+5.0)");
    return;
  }

  String motorName = cmd.substring(0, sep);
  float  val       = cmd.substring(sep + 1).toFloat();

  // Map tên → index
  int idx = -1;
  if      (motorName == "fl") idx = 0;
  else if (motorName == "fr") idx = 1;
  else if (motorName == "bl") idx = 2;
  else if (motorName == "br") idx = 3;
  else {
    bt.println("ERR:TRIM_MOTOR(fl/fr/bl/br)");
    return;
  }

  // Validate range
  if (val < TRIM_MIN || val > TRIM_MAX) {
    char buf[50];
    snprintf(buf, sizeof(buf),
      "ERR:TRIM_RANGE(%.0f to %.0f)", TRIM_MIN, TRIM_MAX);
    bt.println(buf);
    return;
  }

  fc.setMotorTrim(idx, val);

  char buf[50];
  snprintf(buf, sizeof(buf),
    "TRIM:%s=%.1f OK", motorName.c_str(), val);
  bt.println(buf);
  sendTrimStatus();
}

void BTController::sendTrimStatus() {
  char buf[80];
  snprintf(buf, sizeof(buf),
    "TRIM:fl=%.1f,fr=%.1f,bl=%.1f,br=%.1f",
    fc.getMotorTrim(0), fc.getMotorTrim(1),
    fc.getMotorTrim(2), fc.getMotorTrim(3)
  );
  bt.println(buf);
  Serial.println(buf);
}