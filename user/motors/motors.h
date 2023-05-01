#ifndef __MOTORS_H__
#define	__MOTORS_H__

#include "stm32f4xx_hal.h"

void USER_TIM_PWM_Init(void);
void Shot_Control(uint16_t pwmval);
void Servo_Control1(uint16_t angle);
void Servo_Control2(uint16_t angle);
#endif