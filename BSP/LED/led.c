#include "led.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

/********************
  *LED初始化子函数*
********************/
void LED_Init(void)					
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*使能GPIOA和GPIOD时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOD,ENABLE);

	/*初始化GPIOA8,输出速度50MHz，通用推挽输出*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	/*GPIOA8输出高电平,GPIOA8对应于LED0*/
	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	/*初始化GPIOD2,输出速度50MHz，通用推挽输出*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	/*GPIOD2输出高电平,GPIOD2对应于LED1*/
	GPIO_SetBits(GPIOD,GPIO_Pin_2);
}
