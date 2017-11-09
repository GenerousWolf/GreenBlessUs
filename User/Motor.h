#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void TIM8_PWM_Init(void);
void M_Init(void);
void Motor_Speed_Control(s16 motorSpeed,u8 wheelNum);

#endif
