#ifndef __MOTOR_H
#define	__MOTOR_H

#include "stm32f10x.h"
#include "delay.h"
void Motor_PWM_Init(void);
void Motor_PWM_OUTPUT(u16 DR1,u16 DR2,u16 DR3,u16 DR4);
#endif /* __PWM_OUTPUT_H */

