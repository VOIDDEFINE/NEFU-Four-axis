/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：INIT.c
 * 描述    ：系统初始化         
 * 实验平台：HT飞控
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
**********************************************************************************/
#include "include.h"

void IAC_Init(void)
{
  delay_init();			//SYSTICK的时钟固定为HCLK时钟的1/8,SYSCLK:系统时钟为72MHz.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//工程自带，选择中断组2，最高两位用于指定抢占式优先级，最低两位用于指定响应优先级
	uart_init(19200); //uart_for_PC
	USART1_Config(); //Uart_for_Dbus     
	US100_USART3_Config();//Uart_for_HC-SR04
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
	delay_ms(1000);
  MPU6050_Init();
  Motor_PWM_Init(); //周期250hz，
  Timer2_Init(40,7199);//4ms周期，系统控制周期也是4ms
  delay_ms(1000);
  while(Roll == 0)
  {
   MPU6050_Pose();
  }
	PID_controllerInit();
}

void Sensor_Init(void)
{
//	  NRF24L01_Init();
//	 Init_State.MPU6050_State = InitMPU6050();      //MPU6050初始化
//	 Init_State.HMC5883_State = Init_HMC5883L();    //地磁初始化 
//	 Init_State.MS5611_State  = MS5611_init();      //初始化气压计
//	 Init_State.NRF2401_State = NRF24L01_Check();   //检测2401
//	 NRF24L01_Mode(1);  //设置2401为发送模式
	
}
//void State_Display(void)
//{
//  OLED_Fill(0x00); //清屏
//	if(Init_State.MPU6050_State)  OLED_P6x8Str(0,0,"MPU6050 is OK");
// 	else                 OLED_P6x8Str(0,0,"MPU6050 is ERROR");
//	if(Init_State.HMC5883_State)  OLED_P6x8Str(0,1,"HMC5883 is OK");
//	else                 OLED_P6x8Str(0,1,"HMC5883 is ERROR");
//	if(Init_State.MS5611_State)   OLED_P6x8Str(0,2,"MS5611  is OK");
//	else                 OLED_P6x8Str(0,2,"MS5611  is ERROR");
//	if(Init_State.NRF2401_State)  OLED_P6x8Str(0,3,"Nrf2401 is OK");
//	else                 OLED_P6x8Str(0,3,"Nrf2401 is ERROR");
//	
//    OLED_P6x8Str(0,5,"YAW:");
//	OLED_P6x8Str(0,6,"THR:");
//	OLED_P6x8Str(56,5,"ROLL:");
//	OLED_P6x8Str(56,6,"PITCH:"); 
//}


//void  BATTDispaly(void)
//{
//	static u8 ti=0;
//	float Battery_voltage,sum = 0; 

//	BATT[ti] = (float) ADC_ConvertedValue*11/4096*3.3;
//	for(int i=0; i<20; i++)
//	{
//	  sum += BATT[i]; 
//	}
//	Battery_voltage = sum/20;
//	ti++;
//	if(ti == 20)  ti=0;
//	
//    OLED_P6x8Str(0,4,"VBATT:");
//    Dis_Float(4,37,Battery_voltage,1);			//计算电池电压
//	OLED_P6x8Str(62,4,"V");
//	
//	OLED_4num(4,5,Rc_Data.YAW);
//	OLED_4num(4,6,Rc_Data.THROTTLE);
//	OLED_4num(57,5,Rc_Data.ROLL);
//	OLED_4num(58,6,Rc_Data.PITCH);
//}
