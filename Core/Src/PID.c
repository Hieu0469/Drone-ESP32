

#include "PID.h"

// PID Initialization
void PID_Init(PID_Param_t *pid, float Kp, float Ki, float Kd,
		float Ts, float Setpoint, float OutMin, float OutMax,
		int AntiWindup) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->Setpoint = Setpoint;
    pid->PreviousError = 0.0f;
    pid->IntegralSum = 0.0f;
    pid->OutMin = OutMin;
    pid->OutMax = OutMax;
    pid->AntiWindup = AntiWindup;
    pid->prev_time = 0;
}
void PID_SetSamplingTime(PID_Param_t *pid,float Ts){
	pid->Ts = Ts;
}
void PID_SetSetPoint(PID_Param_t *pid,float Setpoint){
	pid->Setpoint = Setpoint;
}
// PID Calculation function
float PID_Calculate(PID_Param_t *pid, float Input) {
    float Error = pid->Setpoint - Input;
    float Proportional = pid->Kp * Error;

    // Khâu Tích phân (I) và chống bão hòa
    pid->IntegralSum += Error * pid->Ts;
    if (pid->AntiWindup) {
        if (pid->IntegralSum > pid->OutMax) pid->IntegralSum = pid->OutMax;
        else if (pid->IntegralSum < pid->OutMin) pid->IntegralSum = pid->OutMin;
    }
    float Integral = pid->Ki * pid->IntegralSum;

    // SỬA LỖI 1: Đạo hàm trên giá trị đo (Chống hiện tượng Derivative Kick)
    // Tận dụng biến PreviousError để lưu Input của vòng lặp trước
    float Derivative = -pid->Kd * (Input - pid->PreviousError) / pid->Ts;
    pid->PreviousError = Input; // Lưu giá trị đo hiện tại cho lần tính sau

    // Tổng hợp tín hiệu
    float Output = Proportional + Integral + Derivative;

    // SỬA LỖI 2: Mở khóa (Uncomment) khâu chặn ngõ ra an toàn
    if (pid->AntiWindup) {
        if (Output > pid->OutMax) Output = pid->OutMax;
        else if (Output < pid->OutMin) Output = pid->OutMin;
    }

    return Output;
}
