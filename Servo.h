// Servo.h
#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);
void Servo_SetPanAngle(float Angle);  // For Horizontal Servo
void Servo_SetTiltAngle(float Angle); // For Vertical Servo

#endif