/**********************************************************************************
 * 文件名  ：Uart_for_Dbus.c
 * 描述    ：UART1用于接收DBUS。配置了dma中断
 *          | PA9  - USART1(Tx)      |
 *          | PA10 - USART1(Rx)      |
 *           ------------------------
**********************************************************************************/
#include "include.h"
#include <stdarg.h>

uint8_t sbus_rx_buffer[18];
uint16_t Channel_Receive[13];
uint8_t key_board[16];

/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置。100000 8-N-1
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* config USART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* USART1 GPIO config */
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 mode config */
    USART_InitStructure.USART_BaudRate = 100000;  //100k
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_Even ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
		DMA_Config(); 
}
/*
 * 函数名：DMA_Config
 * 描述  ：DMA 串口的初始化配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//开启DMA时钟
    NVIC_Config();	   			//配置DMA中断

    /*设置DMA源：内存地址&串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);

    /*内存地址(要传输的变量的指针)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sbus_rx_buffer;

    /*方向：从内存到外设*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

    /*传输大小DMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = 18;

    /*外设地址不增*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*外设数据单位*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*内存数据单位 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMA模式：一次传输，循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;

    /*优先级：中*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    /*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*配置DMA1的4通道*/
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd (DMA1_Channel5,ENABLE);					//使能DMA
    DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);  //配置DMA传输完成后产生中断

}
/*
 * 函数名：NVIC_Config
 * 描述  ：DMA 中断配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
static void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* 配置P[A|B|C|D|E]0为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel5_IRQHandler(void)
{
    //判断是否为DMA发送完成中断
    if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET)
    {
        //遥控器的六个通道解码，每个通道占11个bit，从低到高截取出来
        Channel_Receive[ROLL_D_BUS] = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;
        Channel_Receive[PITCH_D_BUS] = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;
        Channel_Receive[YAW_D_BUS] = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;
        Channel_Receive[THR_D_BUS] = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;
        Channel_Receive[S1] = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;//s1
        Channel_Receive[S2] = ((sbus_rx_buffer[5] >> 4)& 0x0003);//s2
        Channel_Receive[6] = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);//鼠标X轴
        Channel_Receive[7] = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);//鼠标Y轴
        Channel_Receive[8] = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);//鼠标Z轴
        Channel_Receive[9] = sbus_rx_buffer[12];//左键
        Channel_Receive[10] = sbus_rx_buffer[13];//右键
        Channel_Receive[11] = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);//键盘按键1
        Channel_Receive[12] = sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);//NULL

        if(Channel_Receive[S2]==0 || Channel_Receive[S2]>3)
        {
            DMA_Cmd (DMA1_Channel5,DISABLE);
            USART1_Config();//DBUS串口初始化
            DMA_Config();
        }
        DMA_ClearFlag(DMA1_FLAG_TC5);
    }
}


