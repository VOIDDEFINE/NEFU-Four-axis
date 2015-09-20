#ifndef _HMC5883_H
#define _HMC5883_H
#include "stm32f10x.h"
#define	HMC5883L_ADDRESS   0x3C	  //����������IIC�����еĴӵ�ַ
#define FILTER_NUM 10

u8 Init_HMC5883L(void);
void Multiple_Read_HMC5883L(void);
void init_q(void);
//void write_hmc5883_offest(void);
//void read_hmc5883_offest(void);


extern int16_t x,y,z;
extern float  X_HMC, Y_HMC, Z_HMC, MAG_AVGx, MAG_AVGy, MAG_AVGz;
#endif

