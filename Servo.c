// Servo.c
#include "stm32f10x.h" // Device header
#include "PWM.h"

void Servo_Init(void)
{
    PWM_Init();
}

void Servo_SetPanAngle(float Angle)
{
    if (Angle > 270.0f) Angle = 270.0f;
    if (Angle < 0.0f)   Angle = 0.0f;
	
    PWM_SetCompare2(Angle / 270.0f * 2000 + 500);  // 输出500~2500us
}

void Servo_SetTiltAngle(float Angle)
{
    if (Angle > 270.0f) Angle = 270.0f;
    if (Angle < 0.0f)   Angle = 0.0f;
	
    PWM_SetCompare1(Angle / 270.0f * 2000 + 500);  // 输出500~2500us
}


