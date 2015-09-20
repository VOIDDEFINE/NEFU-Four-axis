/**********************************************************************************
 * �ļ���  ��Uart_for_Dbus.c
 * ����    ��UART1���ڽ���DBUS��������dma�ж�
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
 * ��������USART1_Config
 * ����  ��USART1 GPIO ����,����ģʽ���á�100000 8-N-1
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
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
 * ��������DMA_Config
 * ����  ��DMA ���ڵĳ�ʼ������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//����DMAʱ��
    NVIC_Config();	   			//����DMA�ж�

    /*����DMAԴ���ڴ��ַ&�������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);

    /*�ڴ��ַ(Ҫ����ı�����ָ��)*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sbus_rx_buffer;

    /*���򣺴��ڴ浽����*/
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;

    /*�����СDMA_BufferSize=SENDBUFF_SIZE*/
    DMA_InitStructure.DMA_BufferSize = 18;

    /*�����ַ����*/
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /*�ڴ��ַ����*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /*�������ݵ�λ*/
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /*�ڴ����ݵ�λ 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /*DMAģʽ��һ�δ��䣬ѭ��*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;

    /*���ȼ�����*/
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

    /*��ֹ�ڴ浽�ڴ�Ĵ���	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    /*����DMA1��4ͨ��*/
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd (DMA1_Channel5,ENABLE);					//ʹ��DMA
    DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�

}
/*
 * ��������NVIC_Config
 * ����  ��DMA �ж�����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
static void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* ����P[A|B|C|D|E]0Ϊ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel5_IRQHandler(void)
{
    //�ж��Ƿ�ΪDMA��������ж�
    if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET)
    {
        //ң����������ͨ�����룬ÿ��ͨ��ռ11��bit���ӵ͵��߽�ȡ����
        Channel_Receive[ROLL_D_BUS] = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;
        Channel_Receive[PITCH_D_BUS] = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;
        Channel_Receive[YAW_D_BUS] = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;
        Channel_Receive[THR_D_BUS] = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;
        Channel_Receive[S1] = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;//s1
        Channel_Receive[S2] = ((sbus_rx_buffer[5] >> 4)& 0x0003);//s2
        Channel_Receive[6] = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);//���X��
        Channel_Receive[7] = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);//���Y��
        Channel_Receive[8] = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);//���Z��
        Channel_Receive[9] = sbus_rx_buffer[12];//���
        Channel_Receive[10] = sbus_rx_buffer[13];//�Ҽ�
        Channel_Receive[11] = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);//���̰���1
        Channel_Receive[12] = sbus_rx_buffer[16] | (sbus_rx_buffer[17] << 8);//NULL

        if(Channel_Receive[S2]==0 || Channel_Receive[S2]>3)
        {
            DMA_Cmd (DMA1_Channel5,DISABLE);
            USART1_Config();//DBUS���ڳ�ʼ��
            DMA_Config();
        }
        DMA_ClearFlag(DMA1_FLAG_TC5);
    }
}


