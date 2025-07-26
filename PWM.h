#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare); // For Tilt Servo (PA0)
void PWM_SetCompare2(uint16_t Compare); // For Pan Servo (PA1)


#endif
