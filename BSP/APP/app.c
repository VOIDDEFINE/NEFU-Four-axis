/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��INIT.c
 * ����    ��ϵͳ��ʼ��         
 * ʵ��ƽ̨��HT�ɿ�
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
**********************************************************************************/
#include "include.h"

void IAC_Init(void)
{
  delay_init();			//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8,SYSCLK:ϵͳʱ��Ϊ72MHz.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����Դ���ѡ���ж���2�������λ����ָ����ռʽ���ȼ��������λ����ָ����Ӧ���ȼ�
	uart_init(19200); //uart_for_PC
	USART1_Config(); //Uart_for_Dbus     
	US100_USART3_Config();//Uart_for_HC-SR04
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	
	delay_ms(1000);
  MPU6050_Init();
  Motor_PWM_Init(); //����250hz��
  Timer2_Init(40,7199);//4ms���ڣ�ϵͳ��������Ҳ��4ms
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
//	 Init_State.MPU6050_State = InitMPU6050();      //MPU6050��ʼ��
//	 Init_State.HMC5883_State = Init_HMC5883L();    //�شų�ʼ�� 
//	 Init_State.MS5611_State  = MS5611_init();      //��ʼ����ѹ��
//	 Init_State.NRF2401_State = NRF24L01_Check();   //���2401
//	 NRF24L01_Mode(1);  //����2401Ϊ����ģʽ
	
}
//void State_Display(void)
//{
//  OLED_Fill(0x00); //����
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
//    Dis_Float(4,37,Battery_voltage,1);			//�����ص�ѹ
//	OLED_P6x8Str(62,4,"V");
//	
//	OLED_4num(4,5,Rc_Data.YAW);
//	OLED_4num(4,6,Rc_Data.THROTTLE);
//	OLED_4num(57,5,Rc_Data.ROLL);
//	OLED_4num(58,6,Rc_Data.PITCH);
//}
