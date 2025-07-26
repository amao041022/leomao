#include "pid.h"

void PID_Init(PID* pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0;
    pid->prev_error = 0;
    pid->integral = 0;
}

float PID_Compute(PID* pid, float measured)
{
    float error = pid->setpoint - measured;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
