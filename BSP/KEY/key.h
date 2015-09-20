#ifndef __KEY_H__
#define __KEY_H__
#include "stm32f10x.h"

#define KEY0 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_13)	//KEY0对应于GPIOA13 
#define KEY1 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)	//KEY1对应于GPIOA15
#define WK_UP 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) 	//WK_UP对应于GPIOA0

#define RIGHT 	1
#define MIDDLE	2
#define LEFT 	3

void KEY_Init(void);		//按键初始化子函数
u8 KEY_Scan(u8 mode);		//按键扫描子函数

#endif
