#include "include.h"
#include  <math.h>
#define  FILL_NUM  10

int16_t  x,y,z;
float  X_HMC, Y_HMC, Z_HMC, MAG_AVGx, MAG_AVGy, MAG_AVGz;
float	MAG_X_BUF[FILTER_NUM],MAG_Y_BUF[FILTER_NUM],MAG_Z_BUF[FILTER_NUM];


u8 Init_HMC5883L(void)
{
	 u8 date;
   date = Single_Write(HMC5883L_ADDRESS,0x02,0x00);
   return date; 		 
}
/*void read_hmc5883_offest(void)
{
  EE_ReadVariable(VirtAddVarTab[3], &x_offest);
	EE_ReadVariable(VirtAddVarTab[4], &y_offest);
	EE_ReadVariable(VirtAddVarTab[5], &z_offest);
}

void write_hmc5883_offest()
{
  EE_WriteVariable(VirtAddVarTab[3],x_offest);  //д��FLASH
	EE_WriteVariable(VirtAddVarTab[4],y_offest);  //д��FLASH
	EE_WriteVariable(VirtAddVarTab[5],z_offest);  //д��FLASH
}  */

//******************************************************
//��������HMC5883�ڲ��Ƕ����ݣ���ַ��Χ0x3~0x5
//******************************************************

void Multiple_Read_HMC5883L(void)
{      	
 	 u8 i;
	 uint8_t BUF[8];
	 static uint8_t filter_cnt1=0;
	 float temp1=0,temp2=0,temp3=0;
	
    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(HMC5883L_ADDRESS);                   //�����豸��ַ+д�ź�
	  I2C_WaitAck();
    I2C_SendByte(0x03);                   //���ʹ洢��Ԫ��ַ����0x3��ʼ	
	  I2C_WaitAck();
    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(HMC5883L_ADDRESS+1);     //�����豸��ַ+���ź�
	  I2C_WaitAck();
	  for (i=0; i<6; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
    {
        BUF[i] = I2C_RadeByte();          //BUF[0]�洢����
        if (i == 5)
           I2C_NoAck();                   //���һ��������Ҫ��NOACK
        else
           I2C_Ack();                     //��ӦACK
    }
    I2C_Stop();                           //ֹͣ�ź�
    I2C_delay();

	  x = BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register;
	  y = BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register;
	  z = BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register;
	  	
	  X_HMC = (float)(x-258) ;
	  Y_HMC = (float)y*1.005256+78.912596;
	  Z_HMC = (float)z*1.094421+1.641632 ;	

	temp1 = temp2 = temp3 = 0;	
	MAG_X_BUF[filter_cnt1] = X_HMC;//���»�����������
	MAG_Y_BUF[filter_cnt1] = Y_HMC;
	MAG_Z_BUF[filter_cnt1] = Z_HMC;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += MAG_X_BUF[i];
		temp2 += MAG_Y_BUF[i];
		temp3 += MAG_Z_BUF[i];
	}
	MAG_AVGx = temp1 / FILTER_NUM;
	MAG_AVGy = temp2 / FILTER_NUM;
	MAG_AVGz = temp3 / FILTER_NUM;
	filter_cnt1++;
	if(filter_cnt1==FILTER_NUM)	filter_cnt1=0;/**/
}

extern float q0 , q1 , q2 , q3 ;  
void init_q()
{
	u8 i;
	int16_t init_ax,init_ay,init_az;
	float init_Yaw, init_Pitch, init_Roll;
	for(i=0;i<15;i++)
	{
		MPU6050_Read();	
		init_ax = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
		init_ay = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
		init_az = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - sensor.acc.quiet.z;
		Multiple_Read_HMC5883L();
	}
	 init_ax=((float)init_ax)/8192.0;
	 init_ay=((float)init_ay)/8192.0;
	 init_az=((float)init_az)/8192.0;
	 init_Roll = atan2(init_ay, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.295780ת��Ϊ�Ƕ�
   	 init_Pitch=  -asin(init_ax);              //init_Pitch = asin(ay / 1);      
	 init_Yaw  =  -atan2(MAG_AVGx*cos(init_Roll) + MAG_AVGy*sin(init_Roll)*sin(init_Pitch) + MAG_AVGz*sin(init_Roll)*cos(init_Pitch),
	                   MAG_AVGy*cos(init_Pitch) - MAG_AVGz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
					            
	//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
	q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
	q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
	q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
	q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw
	angle.roll=init_Roll* RtA;
	angle.pitch=init_Pitch* RtA;
	angle.yaw=init_Yaw* RtA;
}	   	
 


