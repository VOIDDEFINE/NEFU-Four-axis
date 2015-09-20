/**********************************************************************************
 * 文件名  ：Uart_for_HC-SR04.c
 * 描述    ：UART3用于超声波。
 *          | PB10  - USART3(Tx)      |
 *          | PB11 - USART3(Rx)      |
 *           串口中断里面接收超声波高低两个字节，赋给 Distance 变量
             
**********************************************************************************/
#include "Uart_for_Hc_sr04.h"

extern u8 US100_Flag;
extern int Distance;

void USART3_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure the NVIC Preemption Priority Bits */
    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void US100_USART3_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* config USART3 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

    /* USART3 GPIO config */
    /* Configure USART3 Tx (PB.10) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* Configure USART3 Rx (PB.11) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* USART1 mode config */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    /* 使能串口1接收中断 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART3, ENABLE);

    USART3_NVIC();
}



/// 配置USART1接收中断
void USART3_IRQHandler(void)
{
    uint8_t ch[2];
    uint16_t temp[2];
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        if(US100_Flag == 0)
        {
            ch[0] = USART_ReceiveData(USART3);
            US100_Flag = 2;
        }
        else if(US100_Flag == 2)
        {
            ch[1] = USART_ReceiveData(USART3);
            US100_Flag = 1;
            temp[0] = ch[0];
            temp[1] = ch[1];
            Distance = (temp[0]<<8) + temp[1];
        }
    }

}



