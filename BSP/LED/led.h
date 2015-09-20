#ifndef __LED_H__
#define __LED_H__
#include "sys.h"

#define LED0 PAout(8)	//LED0对应于GPIOA8
#define LED1 PDout(2)	//LED1对应于GPIOD2

void LED_Init(void);	//LED初始化子函数

#endif
