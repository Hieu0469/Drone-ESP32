#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "filter.h"

// =============================================
// CẤU HÌNH
// =============================================
#define SDA_PIN       21
#define SCL_PIN       22
#define I2C_FREQ      400000   // 400kHz Fast Mode
#define DT            0.001f   // 1ms

// =============================================
// DỮ LIỆU THÔ
// =============================================
struct RawData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

// =============================================
// DỮ LIỆU SAU XỬ LÝ
// =============================================
struct SensorData {
  // Gia tốc (m/s²)
  float ax, ay, az;
  // Vận tốc góc (deg/s)
  float gx, gy, gz;
  // Góc (deg)
  float pitch, roll;
};

// =============================================
// CLASS MPU6050 HANDLER
// =============================================
class MPU6050Handler {
public:
  MPU6050Handler()
    : lpf_ax(0.15f), lpf_ay(0.15f), lpf_az(0.15f),
      lpf_gx(0.10f), lpf_gy(0.10f), lpf_gz(0.10f),
      compPitch(0.98f, DT), compRoll(0.98f, DT)
  {}

  bool begin() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);

    mpu.initialize();
    if (!mpu.testConnection()) return false;

    // ±2g, ±250°/s, DLPF 188Hz
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(MPU6050_DLPF_BW_188);

    calibrate(500);  // lấy 500 mẫu để tính offset
    return true;
  }

  // Gọi trong Timer ISR hoặc task
  void update() {
    mpu.getMotion6(
      &raw.ax, &raw.ay, &raw.az,
      &raw.gx, &raw.gy, &raw.gz
    );

    convertRaw();
    applyLowPass();
    computeAngles();
  }

  const SensorData& getData() const { return data; }

  void resetFilters() {
    lpf_ax.reset(); lpf_ay.reset(); lpf_az.reset();
    lpf_gx.reset(); lpf_gy.reset(); lpf_gz.reset();
    compPitch.reset(); compRoll.reset();
    kalmanPitch.reset(); kalmanRoll.reset();
  }

private:
  MPU6050 mpu;
  RawData raw;
  SensorData data;

  // Offset từ calibration
  float off_ax = 0, off_ay = 0, off_az = 0;
  float off_gx = 0, off_gy = 0, off_gz = 0;

  // Bộ lọc
  LowPassFilter lpf_ax, lpf_ay, lpf_az;
  LowPassFilter lpf_gx, lpf_gy, lpf_gz;
  ComplementaryFilter compPitch, compRoll;
  KalmanFilter kalmanPitch, kalmanRoll;

  // ------------------------------------------
  // Chuyển raw → đơn vị thực & trừ offset
  // ------------------------------------------
  void convertRaw() {
    data.ax = (raw.ax / 16384.0f) * 9.81f - off_ax;
    data.ay = (raw.ay / 16384.0f) * 9.81f - off_ay;
    data.az = (raw.az / 16384.0f) * 9.81f - off_az;
    data.gx = raw.gx / 131.0f - off_gx;
    data.gy = raw.gy / 131.0f - off_gy;
    data.gz = raw.gz / 131.0f - off_gz;
  }

  // ------------------------------------------
  // Low Pass Filter
  // ------------------------------------------
  void applyLowPass() {
    data.ax = lpf_ax.update(data.ax);
    data.ay = lpf_ay.update(data.ay);
    data.az = lpf_az.update(data.az);
    data.gx = lpf_gx.update(data.gx);
    data.gy = lpf_gy.update(data.gy);
    data.gz = lpf_gz.update(data.gz);
  }

  // ------------------------------------------
  // Tính góc: Complementary + Kalman
  // ------------------------------------------
  void computeAngles() {
    float accelPitch = atan2f(data.ay, data.az) * 180.0f / PI;
    float accelRoll  = atan2f(-data.ax, data.az) * 180.0f / PI;

    // Complementary
    // data.pitch = compPitch.update(data.gx, accelPitch);
    // data.roll  = compRoll.update(data.gy, accelRoll);

    // Kalman (bỏ comment nếu muốn dùng thay Complementary)
    data.pitch = kalmanPitch.update(data.gx, accelPitch, DT);
    data.roll  = kalmanRoll.update(data.gy, accelRoll, DT);
  }

  // ------------------------------------------
  // Calibration: đặt sensor nằm phẳng, không động
  // ------------------------------------------
  void calibrate(int samples) {
    Serial.println("Calibrating... Giữ yên MPU6050!");
    double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;

    for (int i = 0; i < samples; i++) {
      int16_t ax,ay,az,gx,gy,gz;
      mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
      sax += ax; say += ay; saz += az;
      sgx += gx; sgy += gy; sgz += gz;
      delay(2);
    }

    off_ax = (sax/samples) / 16384.0f * 9.81f;
    off_ay = (say/samples) / 16384.0f * 9.81f;
    off_az = (saz/samples) / 16384.0f * 9.81f - 9.81f; // trừ gravity
    off_gx = (sgx/samples) / 131.0f;
    off_gy = (sgy/samples) / 131.0f;
    off_gz = (sgz/samples) / 131.0f;

    Serial.printf("Offset ax:%.4f ay:%.4f az:%.4f\n",
                  off_ax, off_ay, off_az);
    Serial.printf("Offset gx:%.4f gy:%.4f gz:%.4f\n",
                  off_gx, off_gy, off_gz);
    Serial.println("Calibration done!");
  }
};