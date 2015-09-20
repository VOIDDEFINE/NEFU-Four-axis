#include "key.h"
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

/********************
  *按键初始化子函数*
********************/
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*使能GPIOA时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	/*初始化GPIOA13、GPIOA15,上拉输入*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	/*初始化GPIOA0,下拉输入*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

/***********************************************************************
  *按键扫描子函数*
  *mode=1：支持连续按下，mode=0：不支持连续按下*
  *返回值：KEY0按下返回1,KEY1按下返回2,WK_UP按下返回3,不按任何按键返回0*
  *注意此函数有响应优先级,KEY0>KEY1>KEY2>KEY3!!
************************************************************************/
u8 KEY_Scan(u8 mode)
{
	/*按键松手标志，用于不支持连续按下模式，等于0的时候表示已经松手*/
	static u8 key_up=1;

	/*mode=1的时候支持按键连续按下*/
	if(mode)
		key_up=1;

	/*开始按键扫描的时候,禁用JTAG和SWD,是为了使得GPIOA13和GPIOA15
	 *作为普通IO口使用,STM32在默认状态下是开启JTAG的*/
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
	{
		delay_ms(10);		//延时去抖
		key_up=0;			//按键松手标志
		if(KEY0==0)			//KEY0按下
		{
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,DISABLE);//开启JTAG和SWD
			return RIGHT;	
		}
		else if(KEY1==0)	//KEY1按下
		{
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,DISABLE);//开启JTAG和SWD
			return MIDDLE;
		}
		else if(WK_UP==1)	//WK_UP按下
		{
			GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,DISABLE);//开启JTAG和SWD
			return LEFT;
		}	
	}
	else if(KEY0==1&&KEY1==1&&WK_UP==0)		//松手检测
		key_up=1;
	
	return 0;								//无按键按下	
}
