#ifndef __UART_FOR_DBUS_H
#define	__UART_FOR_DBUS_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void DMA_Config(void);
static void NVIC_Config(void);

extern uint16_t Channel_Receive[13];
extern uint8_t key_board[16];

typedef enum
{
    ROLL_D_BUS,
    PITCH_D_BUS,
    YAW_D_BUS,
    THR_D_BUS,//смце
    S1,
    S2,
    MOUSE_X_POS,
    MOUSE_Y_POS,
    MOUSE_Z_POS,
    MOUSE_LEFT,
    MOUSE_RIGHT,
    KEYBOARD,
    NULL_D_Bus,
} HAND_CONTROL_CASE;

typedef enum
{
    KEY_W,
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
} KEY_BOARD;


#endif /* __USART1_H */
