#ifndef __UART_FOR_PC_H
#define __UART_FOR_PC_H
#include "stdio.h"
#include "stm32f10x.h"

//////////////////////////////////////////////////////////////////////////////////
extern u8 USART_RX_BUF[64];     //接收缓冲,最大63个字节.末字节为换行符
extern u8 USART_RX_STA;         //接收状态标记

void uart_init(u32 bound);
void PrintChar(char *s);
void UsartSend(u16 ch);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
                     ,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void CRC_send(int *s);
#endif
