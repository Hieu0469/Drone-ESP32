<<<<<<< HEAD
#include "stm32f4xx_hal.h"
typedef struct {
    float Kp;         // Proportional gain
    float Ki;         // Integral gain
    float Kd;         // Derivative gain
    float Ts;         // Sampling time (update interval)
    float Setpoint;   // Desired value
    float PreviousError; // Previous error for derivative calculation
    float IntegralSum;   // Sum of errors for integral calculation
    float OutMin;     // Minimum output limit
    float OutMax;     // Maximum output limit
    int AntiWindup;   // Anti-windup flag (0 or 1)
    uint32_t prev_time;
} PID_Param_t;

// PID Initialization
void PID_Init(PID_Param_t *pid, float Kp, float Ki, float Kd, float Ts,
		float Setpoint, float OutMin, float OutMax, int AntiWindup);


// PID Calculation function
float PID_Calculate(PID_Param_t *pid, float Input);

void PID_SetSamplingTime(PID_Param_t *pid,float Ts);

void PID_SetSetPoint(PID_Param_t *pid,float Setpoint);
=======
#include "stm32f4xx_hal.h"
typedef struct {
    float Kp;         // Proportional gain
    float Ki;         // Integral gain
    float Kd;         // Derivative gain
    float Ts;         // Sampling time (update interval)
    float Setpoint;   // Desired value
    float PreviousError; // Previous error for derivative calculation
    float IntegralSum;   // Sum of errors for integral calculation
    float OutMin;     // Minimum output limit
    float OutMax;     // Maximum output limit
    int AntiWindup;   // Anti-windup flag (0 or 1)
    uint32_t prev_time;
} PID_Param_t;

// PID Initialization
void PID_Init(PID_Param_t *pid, float Kp, float Ki, float Kd, float Ts,
		float Setpoint, float OutMin, float OutMax, int AntiWindup);


// PID Calculation function
float PID_Calculate(PID_Param_t *pid, float Input);

void PID_SetSamplingTime(PID_Param_t *pid,float Ts);

void PID_SetSetPoint(PID_Param_t *pid,float Setpoint);
>>>>>>> 1afa4d579ae4796debe49eb915d73c25e3f8f8c3
