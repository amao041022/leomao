#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float prev_error;
    float integral;
} PID;

void PID_Init(PID* pid, float Kp, float Ki, float Kd);
float PID_Compute(PID* pid, float measured);

#endif
