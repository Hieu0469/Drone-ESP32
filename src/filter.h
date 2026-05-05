#pragma once

// =============================================
// LOW PASS FILTER
// =============================================
struct LowPassFilter {
  float alpha;
  float output;

  LowPassFilter(float alpha = 0.1f) 
    : alpha(alpha), output(0.0f) {}

  void reset() { output = 0.0f; }

  float update(float input) {
    output = alpha * input + (1.0f - alpha) * output;
    return output;
  }
};

// =============================================
// COMPLEMENTARY FILTER
// =============================================
struct ComplementaryFilter {
  float alpha;  // thường 0.98
  float angle;
  float dt;

  ComplementaryFilter(float alpha = 0.96f, float dt = 0.001f)
    : alpha(alpha), angle(0.0f), dt(dt) {}

  void reset() { angle = 0.0f; }

  // gyroRate: deg/s, accelAngle: deg
  float update(float gyroRate, float accelAngle) {
    angle = alpha * (angle + gyroRate * dt)
          + (1.0f - alpha) * accelAngle;
    return angle;
  }
};

// =============================================
// KALMAN FILTER
// =============================================
struct KalmanFilter {
  float Q_angle;    // 0.001
  float Q_bias;     // 0.003
  float R_measure;  // 0.03

  float angle, bias, rate;
  float P[2][2];

  KalmanFilter(float Q_angle = 0.001f,
               float Q_bias  = 0.003f,
               float R_meas  = 0.03f)
    : Q_angle(Q_angle), Q_bias(Q_bias),
      R_measure(R_meas), angle(0), bias(0), rate(0) {
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  void reset() {
    angle = 0; bias = 0; rate = 0;
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  float update(float gyroRate, float accelAngle, float dt) {
    // --- Predict ---
    rate   = gyroRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // --- Update ---
    float S    = P[0][0] + R_measure;
    float K[2] = { P[0][0] / S, P[1][0] / S };
    float y    = accelAngle - angle;

    angle    += K[0] * y;
    bias     += K[1] * y;
    P[0][0]  -= K[0] * P[0][0];
    P[0][1]  -= K[0] * P[0][1];
    P[1][0]  -= K[1] * P[0][0];
    P[1][1]  -= K[1] * P[0][1];

    return angle;
  }
};