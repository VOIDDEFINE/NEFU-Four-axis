/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��ms5611.c
 * ����    ��ms5611����         
 * ʵ��ƽ̨��HT�ɿ�
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
**********************************************************************************/
#include "include.h"
#include "math.h"

//��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ�¶�ת��
#define CTemperatureing  0x02 //����ת���¶�
#define SCPressure  0x03	    //��ʼ��ѹת��
#define SCPressureing  0x04	  //����ת����ѹ


 
/*
C1 ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
uint32_t  Cal_C[7];	        //���ڴ��PROM�е�6������1-6

double OFF_;
float Aux;
/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�	
*/
uint64_t dT,TEMP;

/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
uint64_t OFf,SENS;
uint32_t D1_Pres,D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ

uint32_t Pressure,Pressure_old,qqp;				//����ѹ
uint32_t TEMP2,T2,OFF2,SENS2;	//�¶�У��ֵ
uint32_t Pres_BUFFER[20];
uint32_t Temp_BUFFER[10];




/*******************************************************************************
  * @��������	MS561101BA_RESET
  * @����˵��   ��λMS5611
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void MS561101BA_RESET(void)
{
  I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(MS561101BA_RST);
	I2C_WaitAck();
	I2C_Stop();
}
/*******************************************************************************
  * @��������	MS5611_init
  * @����˵��   ��ʼ��5611
  * @�������  	��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
u8 MS5611_init(void)
 {	 
   uint8_t d1,d2,i;
   MS561101BA_RESET();	 // Reset Device
	 delay_ms(20);  
	 for(i=1;i<=6;i++)
	 {
		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress);
		I2C_WaitAck();
		I2C_SendByte((MS561101BA_PROM_RD+i*2));
		I2C_WaitAck();
	  I2C_Stop();
		delay_ms(1);

		I2C_Start();
		I2C_SendByte(MS561101BA_SlaveAddress+1);
		I2C_WaitAck();
		d1=I2C_RadeByte();
		I2C_Ack();
		d2=I2C_RadeByte();
		I2C_NoAck();
		I2C_Stop();

		I2C_delay();
		Cal_C[i]=((uint16_t)d1<<8)|d2;
	  delay_ms(10);
	 }
	 
	 return !Cal_C[0];
 }


void MS561101BA_startConversion(uint8_t command) 
{	
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(command);
  I2C_WaitAck();
	I2C_Stop();
}


unsigned long MS561101BA_getConversion(void) 
{
	unsigned long conversion = 0;
	uint8_t conv1,conv2,conv3; 
	
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(0);
	I2C_WaitAck();
  I2C_Stop();


	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress+1);
	I2C_WaitAck();
	conv1=I2C_RadeByte();
	I2C_Ack();
	conv2=I2C_RadeByte();
	I2C_Ack();
	conv3=I2C_RadeByte();

	I2C_NoAck();
	I2C_Stop();

	conversion=conv1*65535+conv2*256+conv3;
	return conversion;
}

/***********************************************
  * @brief  ��ȡ�¶�ת��
  * @param  None
  * @retval None
************************************************/
void MS561101BA_getTemperature(void)
{  
  static u8 p=0;
	uint32_t sum=0,max=0,min=200000000;
	
	Temp_BUFFER[p] = MS561101BA_getConversion();
	p++;
	if(p==100) p=0;
	for(u8 i=0;i<10;i++) 
  {
		if(Temp_BUFFER[i] > max)  max = Temp_BUFFER[i];
		else if(Temp_BUFFER[i] < min)  min = Temp_BUFFER[i];
		sum += Temp_BUFFER[i];
	}	
	D2_Temp =(sum -  max -min)/8;
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;
	
}

/***********************************************
  * @brief  ��ȡ��ѹ
  * @param  None
  * @retval None
************************************************/
void MS561101BA_getPressure(void)
{
	uint32_t sum=0,max=0,min=200000;
	static u8 p=0;
	
	D1_Pres= MS561101BA_getConversion();
	I2C_delay();
	OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
	SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;

	if(TEMP<2000)
	{
		Aux = TEMP*TEMP;
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		TEMP = TEMP - TEMP2;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;	
	}
  Pres_BUFFER[p] = qqp =((D1_Pres*SENS/2097152-OFF_)/32768);
	p++;
	if(p==20) p=0;
	for(u8 i=0;i<20;i++) 
  {
		if(Pres_BUFFER[i] > max)  max = Pres_BUFFER[i];
		else if(Pres_BUFFER[i] < min)  min = Pres_BUFFER[i];
		sum +=Pres_BUFFER[i];
	}	
	Pressure=(sum -  max -min)/18;
}
/***********************************************
  * @brief  �õ��߶�
  * @param  None
  * @retval None
************************************************/
float Get_High(void)//OSRΪѹǿת������
{
	static u8 Now_doing = SCTemperature;
	static u8 flag=0;
	 switch(Now_doing)
	 {
		 case SCTemperature: 
		 {		
			 if(flag)    MS561101BA_getPressure();			 
			 MS561101BA_startConversion(MS561101BA_D2_OSR_4096);  
			 Now_doing = CTemperatureing;//�л�����һ��״̬
		 }
		 break;
		 case CTemperatureing: 
		 {		
			 MS561101BA_getTemperature();
			 MS561101BA_startConversion(MS561101BA_D1_OSR_4096); 
			 flag = 1;
			 Now_doing = SCTemperature;
		 }
		 break;
     default: 
		 {
			 Now_doing = SCTemperature; //������ ���¿�ʼ
			 flag=0;
		 }
 		 break;			 
	 }
	
}

extern u8 Control_Mode;
unsigned int g_Hight=0,g_HightOld=0;
float g_Alt_Hight=0,g_Alt_HightOld=0;
float g_HightControl=0,g_HightControlold=0,g_HightPwm=0,hight_increment=0;
unsigned char RcvIndex,GLengthHigh, GLengthLow;
float g_hight_Kp=0.14,g_hight_Ki=0.005,g_hight_Kd=1;  
void Filter_Hight(unsigned int set_hight)
{
	static float hight_error=0,hight_errorold=0;
	static float Alt_Hight[3];
	RcvIndex = 0; 
	g_Hight = GLengthHigh*256;
	g_Hight += GLengthLow;
	UART2_Put_Char(0X55);
	if(g_Hight>4000||g_Hight<50)
		g_Hight=g_HightOld;
	else
		g_HightOld=g_Hight;

	Alt_Hight[2]=Alt_Hight[1];
	Alt_Hight[1]=Alt_Hight[0];
	Alt_Hight[0]=(float)g_Hight*((float)cos(angle.roll/RtA)*(float)cos(angle.pitch/RtA));
	g_Alt_HightOld=g_Alt_Hight;
	g_Alt_Hight=(Alt_Hight[0]+Alt_Hight[1]+Alt_Hight[2])/3;

	if(g_Alt_Hight-g_Alt_HightOld>100)						   //����������
	    g_Alt_Hight=g_Alt_HightOld+100;
	else if(g_Alt_HightOld-g_Alt_Hight>100)
		g_Alt_Hight=g_Alt_HightOld-100;
//////////////////////////////////////////////////////////
	if(!Control_Mode||!ARMED)
		hight_increment=0;
	hight_errorold=hight_error;
	hight_error=set_hight-g_Alt_Hight;
	hight_increment+=hight_error;

	if(hight_increment>64200)
		hight_increment=64200;
	else if(hight_increment<0)
		hight_increment=0;
	g_HightControlold=g_HightControl;
	g_HightControl=g_hight_Kp*hight_error+g_hight_Ki*hight_increment+g_hight_Kd*(hight_error-hight_errorold);
	if(g_HightControl-g_HightControlold>50)						   //��ͻ��
	    g_HightControl=g_HightControlold+50;
	else if(g_HightControlold-g_HightControl>50)
		g_HightControl=g_HightControlold-50;
	if(g_HightControl>600)
		g_HightControl=600;
	else if(g_HightControl<0)
		g_HightControl=0;
}

extern u8  ms2;
void Hight_PwmOut()
{
	g_HightPwm=g_HightControlold+(g_HightControl-g_HightControlold)*(float)ms2/30.0;	
}


