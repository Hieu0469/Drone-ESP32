<<<<<<< HEAD


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
    pid->IntegralSum +=  Error  * pid->Ts;
    if (pid->AntiWindup) { // Basic anti-windup
        if (pid->IntegralSum > pid->OutMax) pid->IntegralSum = pid->OutMax;
        else if (pid->IntegralSum < pid->OutMin) pid->IntegralSum = pid->OutMin;
    }
    float Integral = pid->Ki * pid->IntegralSum;
    float Derivative;

    Derivative = pid->Kd * (Error - pid->PreviousError) / pid->Ts;
    pid->PreviousError = Error;

    float Output = Proportional + Integral + Derivative;

    // Output clamping
//    if(pid->AntiWindup){
//        if (Output > pid->OutMax) Output = pid->OutMax;
//        else if (Output < pid->OutMin) Output = pid->OutMin;
//    }


    return Output;
}
=======


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
    pid->IntegralSum +=  Error  * pid->Ts;
    if (pid->AntiWindup) { // Basic anti-windup
        if (pid->IntegralSum > pid->OutMax) pid->IntegralSum = pid->OutMax;
        else if (pid->IntegralSum < pid->OutMin) pid->IntegralSum = pid->OutMin;
    }
    float Integral = pid->Ki * pid->IntegralSum;
    float Derivative;

    Derivative = pid->Kd * (Error - pid->PreviousError) / pid->Ts;
    pid->PreviousError = Error;

    float Output = Proportional + Integral + Derivative;

    // Output clamping
//    if(pid->AntiWindup){
//        if (Output > pid->OutMax) Output = pid->OutMax;
//        else if (Output < pid->OutMin) Output = pid->OutMin;
//    }


    return Output;
}
>>>>>>> 1afa4d579ae4796debe49eb915d73c25e3f8f8c3
