<<<<<<< HEAD
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c1;
#define PI 3.14159265358979323846f
#define MPU6050_ADDR (0x68 << 1)  // địa chỉ I2C của MPU6050
#define SMPLRT_DIV_REG        0x19
#define CONFIG_REG            0x1A
#define GYRO_CONFIG_REG       0x1B
#define ACCEL_CONFIG_REG      0x1C
#define ACCEL_CONFIG_REG_2    0x1D
#define WHO_AM_I_REG          0x75
#define PWR_MGMT_1_REG        0x6B
#define ACCEL_XOUT_H_REG      0x3B
#define GYRO_XOUT_H_REG 	  0x43

#define timeOut 1000

typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;

	float Ax;
	float Ay;
	float Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;

	float Gx;
	float Gy;
	float Gz;

	float Temp;
} MPU6050_Raw;

=======
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c1;
#define PI 3.14159265358979323846f
#define MPU6050_ADDR (0x68 << 1)  // địa chỉ I2C của MPU6050
#define SMPLRT_DIV_REG        0x19
#define CONFIG_REG            0x1A
#define GYRO_CONFIG_REG       0x1B
#define ACCEL_CONFIG_REG      0x1C
#define ACCEL_CONFIG_REG_2    0x1D
#define WHO_AM_I_REG          0x75
#define PWR_MGMT_1_REG        0x6B
#define ACCEL_XOUT_H_REG      0x3B
#define GYRO_XOUT_H_REG 	  0x43

#define timeOut 1000

typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;

	float Ax;
	float Ay;
	float Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;

	float Gx;
	float Gy;
	float Gz;

	float Temp;
} MPU6050_Raw;

>>>>>>> 1afa4d579ae4796debe49eb915d73c25e3f8f8c3
extern uint8_t TxBuffer[];
extern uint8_t TxBufferLen;
extern char msg[64];
extern uint16_t samples;
extern float gyro_x_offset, gyro_y_offset, gyro_z_offset;
extern uint8_t calibration_done;
extern float angle_roll;
extern float angle_pitch; // Góc sau khi lọc
extern float accel_roll;
extern float accel_pitch; // Góc tính từ gia tốc
extern float accel_roll_filtered;
extern float accel_pitch_filtered;
extern float alpha;
<<<<<<< HEAD
extern float accel_alpha;

void MPU6050_Init(void);

void MPU6050_Read_Data(MPU6050_Raw *Raw);

void MPU6050_Process_Angle(MPU6050_Raw *Data);
=======
extern float accel_alpha;

void MPU6050_Init(void);

void MPU6050_Read_Data(MPU6050_Raw *Raw);

void MPU6050_Process_Angle(MPU6050_Raw *Data);
>>>>>>> 1afa4d579ae4796debe49eb915d73c25e3f8f8c3
