<<<<<<< HEAD

#include "MPU.h"



uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
uint8_t TxBufferLen = sizeof(TxBuffer);
char msg[64];
uint16_t samples = 0;


float angle_roll = 0, angle_pitch = 0; // Góc sau khi lọc
float accel_roll = 0, accel_pitch = 0; // Góc tính từ gia tốc
float accel_roll_filtered = 0, accel_pitch_filtered = 0; // Góc accelerometer sau khi lọc
uint32_t last_time = 0;                // Để tính dt
float alpha = 0.98;                    // Hệ số lọc complementary (thường từ 0.95 - 0.99)
float accel_alpha = 0.5;              // Hệ số lọc cho accelerometer (giảm nhiễu)

// Biến bù sai số (Calibration) - PHẢI bắt đầu từ 0
float gyro_x_offset = 0, gyro_y_offset = 1.5, gyro_z_offset = 0;
uint8_t calibration_done = 0;         // Cờ báo calibration hoàn thành

#include <math.h>

void MPU6050_Process_Angle(MPU6050_Raw *Data) {
    // Không tính góc nếu calibration chưa xong
    if (!calibration_done) {
        return;
    }
    
    // 1. Tính toán thời gian giữa 2 lần đọc (dt)
    uint32_t current_time = HAL_GetTick();
    
    // Khởi tạo last_time lần đầu tiên
    if (last_time == 0) {
        last_time = current_time;
        return;
    }
    
    float dt = (current_time - last_time) / 1000.0f; // Đổi sang giây
    last_time = current_time;
    
    // Bảo vệ: nếu dt quá lớn (>0.1s) hoặc quá nhỏ → bỏ qua
    if (dt > 0.1f || dt <= 0.0f) {
        return;
    }

    // 2. Tính góc từ Gia tốc (Accelerometer)
    // Công thức lượng giác: atan2 trả về radian, sau đó đổi sang độ (* 180 / PI)
    accel_roll = atan2(Data->Ay, Data->Az) * 57.29578f;
    accel_pitch = atan2(-Data->Ax, sqrt(Data->Ay * Data->Ay + Data->Az * Data->Az)) * 57.29578f;
    
    // 2.5. Lọc nhiễu accelerometer bằng low-pass filter
    accel_roll_filtered = accel_alpha * accel_roll + (1.0f - accel_alpha) * accel_roll_filtered;
    accel_pitch_filtered = accel_alpha * accel_pitch + (1.0f - accel_alpha) * accel_pitch_filtered;

    // 3. Bộ lọc bù (Complementary Filter)
    // Góc = Alpha * (Góc_cũ + Vận_tốc_góc * dt) + (1 - Alpha) * Góc_gia_tốc
    // Lưu ý: Gyro offset đã được trừ trong MPU6050_Read_Data
    angle_roll = alpha * (angle_roll + Data->Gx * dt) + (1.0f - alpha) * accel_roll_filtered;
    angle_pitch = alpha * (angle_pitch + Data->Gy * dt) + (1.0f - alpha) * accel_pitch_filtered;

    // 4. (Tùy chọn) Gửi dữ liệu lên máy tính để vẽ biểu đồ (Serial Plotter)
//    char buf[100];
//    sprintf(buf, "%.2f,%.2f\r\n", angle_roll, angle_pitch);
//    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, timeOut);

    if (check == 104)  // 0x68
    {
        // Wake up MPU6050
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, timeOut);
        HAL_Delay(100); // Đợi MPU6050 ổn định

        // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, timeOut);

        // **QUAN TRỌNG**: Cấu hình DLPF (Digital Low Pass Filter)
        // 0x03 = 44Hz bandwidth (giảm nhiễu tần số cao rất tốt)
        Data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, timeOut);

        // Set accelerometer range ±4g
        Data = 0x08;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, timeOut);

        // Set gyroscope range ±250°/s (chính xác nhất)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, timeOut);
    }
    else
    {
        sprintf(msg, "MPU6050 not found!\r\n");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        HAL_Delay(500);
    }
}
void MPU6050_Read_Data(MPU6050_Raw *Raw){

	uint8_t Rec_Data[14];
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,timeOut);

	Raw->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Raw->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Raw->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Ghép byte cho Temperature
	Raw->Temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

	// Ghép byte cho Gyroscope (Index tiếp theo trong mảng 14 byte)
	Raw->Gyro_X_RAW = (int16_t)(Rec_Data[8]  << 8 | Rec_Data[9]);
	Raw->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	Raw->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g  ~  0000 0000  ~ 0x00 16,384
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 4g  ~  0000 1000  ~ 0x08 8192
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 8g  ~  0001 0000  ~ 0x10 4096
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 16g ~  0001 1000  ~ 0x18 2048 		 * ****/
	Raw->Ax = Raw->Accel_X_RAW/8192.0; //16384.0;
	Raw->Ay = Raw->Accel_Y_RAW/8192.0;
	Raw->Az = Raw->Accel_Z_RAW/8192.0;

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s      ~  0000 0000  ~ 0x00 131
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> � 500 �/s      ~  0000 1000  ~ 0x08 65.5
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=2 -> � 1000 �/s   ~  0001 0000  ~ 0x10 32.8
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=3 -> � 2000 �/s   ~  0001 1000  ~ 0x18 16.4  			****/
		// Calibration: bỏ qua 32 mẫu đầu, lấy trung bình 100 mẫu tiếp theo
		if(samples < 32) {
		  samples++;
		  return;
		} else if(samples < 132) {  // Lấy 100 mẫu để calibration chính xác hơn
			gyro_x_offset += Raw->Gyro_X_RAW;
			gyro_y_offset += Raw->Gyro_Y_RAW;
			gyro_z_offset += Raw->Gyro_Z_RAW;
			samples++;
			return;
		} else if(samples == 132) {
			// Tính trung bình 100 mẫu
			gyro_x_offset /= 100.0f;
			gyro_y_offset /= 100.0f;
			gyro_z_offset /= 100.0f;
			samples++;
			calibration_done = 1;  // Đánh dấu calibration hoàn thành
			
			// Khởi tạo góc ban đầu từ accelerometer
			accel_roll = atan2(Raw->Ay, Raw->Az) * 57.29578f;
			accel_pitch = atan2(-Raw->Ax, sqrt(Raw->Ay * Raw->Ay + Raw->Az * Raw->Az)) * 57.29578f;
			angle_roll = accel_roll;
			angle_pitch = accel_pitch;
			accel_roll_filtered = accel_roll;
			accel_pitch_filtered = accel_pitch;
			
			return;
		} else {
			// Trừ offset sau khi calibration xong
			Raw->Gyro_X_RAW -= (int16_t)gyro_x_offset;
			Raw->Gyro_Y_RAW -= (int16_t)gyro_y_offset;
			Raw->Gyro_Z_RAW -= (int16_t)gyro_z_offset;
		}
	Raw->Gx = Raw->Gyro_X_RAW/131.0;
	Raw->Gy = Raw->Gyro_Y_RAW/131.0;
	Raw->Gz = Raw->Gyro_Z_RAW/131.0;

}
=======

#include "MPU.h"



uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
uint8_t TxBufferLen = sizeof(TxBuffer);
char msg[64];
uint16_t samples = 0;


float angle_roll = 0, angle_pitch = 0; // Góc sau khi lọc
float accel_roll = 0, accel_pitch = 0; // Góc tính từ gia tốc
float accel_roll_filtered = 0, accel_pitch_filtered = 0; // Góc accelerometer sau khi lọc
uint32_t last_time = 0;                // Để tính dt
float alpha = 0.98;                    // Hệ số lọc complementary (thường từ 0.95 - 0.99)
float accel_alpha = 0.5;              // Hệ số lọc cho accelerometer (giảm nhiễu)

// Biến bù sai số (Calibration) - PHẢI bắt đầu từ 0
float gyro_x_offset = 0, gyro_y_offset = 1.5, gyro_z_offset = 0;
uint8_t calibration_done = 0;         // Cờ báo calibration hoàn thành

#include <math.h>

void MPU6050_Process_Angle(MPU6050_Raw *Data) {
    // Không tính góc nếu calibration chưa xong
    if (!calibration_done) {
        return;
    }
    
    // 1. Tính toán thời gian giữa 2 lần đọc (dt)
    uint32_t current_time = HAL_GetTick();
    
    // Khởi tạo last_time lần đầu tiên
    if (last_time == 0) {
        last_time = current_time;
        return;
    }
    
    float dt = (current_time - last_time) / 1000.0f; // Đổi sang giây
    last_time = current_time;
    
    // Bảo vệ: nếu dt quá lớn (>0.1s) hoặc quá nhỏ → bỏ qua
    if (dt > 0.1f || dt <= 0.0f) {
        return;
    }

    // 2. Tính góc từ Gia tốc (Accelerometer)
    // Công thức lượng giác: atan2 trả về radian, sau đó đổi sang độ (* 180 / PI)
    accel_roll = atan2(Data->Ay, Data->Az) * 57.29578f;
    accel_pitch = atan2(-Data->Ax, sqrt(Data->Ay * Data->Ay + Data->Az * Data->Az)) * 57.29578f;
    
    // 2.5. Lọc nhiễu accelerometer bằng low-pass filter
    accel_roll_filtered = accel_alpha * accel_roll + (1.0f - accel_alpha) * accel_roll_filtered;
    accel_pitch_filtered = accel_alpha * accel_pitch + (1.0f - accel_alpha) * accel_pitch_filtered;

    // 3. Bộ lọc bù (Complementary Filter)
    // Góc = Alpha * (Góc_cũ + Vận_tốc_góc * dt) + (1 - Alpha) * Góc_gia_tốc
    // Lưu ý: Gyro offset đã được trừ trong MPU6050_Read_Data
    angle_roll = alpha * (angle_roll + Data->Gx * dt) + (1.0f - alpha) * accel_roll_filtered;
    angle_pitch = alpha * (angle_pitch + Data->Gy * dt) + (1.0f - alpha) * accel_pitch_filtered;

    // 4. (Tùy chọn) Gửi dữ liệu lên máy tính để vẽ biểu đồ (Serial Plotter)
//    char buf[100];
//    sprintf(buf, "%.2f,%.2f\r\n", angle_roll, angle_pitch);
//    CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
}
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, timeOut);

    if (check == 104)  // 0x68
    {
        // Wake up MPU6050
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, timeOut);
        HAL_Delay(100); // Đợi MPU6050 ổn định

        // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, timeOut);

        // **QUAN TRỌNG**: Cấu hình DLPF (Digital Low Pass Filter)
        // 0x03 = 44Hz bandwidth (giảm nhiễu tần số cao rất tốt)
        Data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, timeOut);

        // Set accelerometer range ±4g
        Data = 0x08;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, timeOut);

        // Set gyroscope range ±250°/s (chính xác nhất)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, timeOut);
    }
    else
    {
        sprintf(msg, "MPU6050 not found!\r\n");
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
        HAL_Delay(500);
    }
}
void MPU6050_Read_Data(MPU6050_Raw *Raw){

	uint8_t Rec_Data[14];
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,timeOut);

	Raw->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Raw->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Raw->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	// Ghép byte cho Temperature
	Raw->Temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

	// Ghép byte cho Gyroscope (Index tiếp theo trong mảng 14 byte)
	Raw->Gyro_X_RAW = (int16_t)(Rec_Data[8]  << 8 | Rec_Data[9]);
	Raw->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	Raw->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g  ~  0000 0000  ~ 0x00 16,384
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 4g  ~  0000 1000  ~ 0x08 8192
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 8g  ~  0001 0000  ~ 0x10 4096
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 16g ~  0001 1000  ~ 0x18 2048 		 * ****/
	Raw->Ax = Raw->Accel_X_RAW/8192.0; //16384.0;
	Raw->Ay = Raw->Accel_Y_RAW/8192.0;
	Raw->Az = Raw->Accel_Z_RAW/8192.0;

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s      ~  0000 0000  ~ 0x00 131
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> � 500 �/s      ~  0000 1000  ~ 0x08 65.5
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=2 -> � 1000 �/s   ~  0001 0000  ~ 0x10 32.8
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=3 -> � 2000 �/s   ~  0001 1000  ~ 0x18 16.4  			****/
		// Calibration: bỏ qua 32 mẫu đầu, lấy trung bình 100 mẫu tiếp theo
		if(samples < 32) {
		  samples++;
		  return;
		} else if(samples < 132) {  // Lấy 100 mẫu để calibration chính xác hơn
			gyro_x_offset += Raw->Gyro_X_RAW;
			gyro_y_offset += Raw->Gyro_Y_RAW;
			gyro_z_offset += Raw->Gyro_Z_RAW;
			samples++;
			return;
		} else if(samples == 132) {
			// Tính trung bình 100 mẫu
			gyro_x_offset /= 100.0f;
			gyro_y_offset /= 100.0f;
			gyro_z_offset /= 100.0f;
			samples++;
			calibration_done = 1;  // Đánh dấu calibration hoàn thành
			
			// Khởi tạo góc ban đầu từ accelerometer
			accel_roll = atan2(Raw->Ay, Raw->Az) * 57.29578f;
			accel_pitch = atan2(-Raw->Ax, sqrt(Raw->Ay * Raw->Ay + Raw->Az * Raw->Az)) * 57.29578f;
			angle_roll = accel_roll;
			angle_pitch = accel_pitch;
			accel_roll_filtered = accel_roll;
			accel_pitch_filtered = accel_pitch;
			
			return;
		} else {
			// Trừ offset sau khi calibration xong
			Raw->Gyro_X_RAW -= (int16_t)gyro_x_offset;
			Raw->Gyro_Y_RAW -= (int16_t)gyro_y_offset;
			Raw->Gyro_Z_RAW -= (int16_t)gyro_z_offset;
		}
	Raw->Gx = Raw->Gyro_X_RAW/131.0;
	Raw->Gy = Raw->Gyro_Y_RAW/131.0;
	Raw->Gz = Raw->Gyro_Z_RAW/131.0;

}
>>>>>>> 1afa4d579ae4796debe49eb915d73c25e3f8f8c3
