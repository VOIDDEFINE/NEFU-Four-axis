#include "include.h" 
extern uint16_t Channel_Receive[13];
extern float thr_adder_Integration;
volatile uint8_t tim2flag=0;
u8 US100_Flag = 1;
int Distance = 0;
int main(void)
{
    uint8_t Tim100ms = 0;
    int CRC2[4]= {0};
    IAC_Init();

    while(1)
    {
        if(US100_Flag == 1)//³¬Éù²¨²É¼¯ÖÐ¶Ï
        {
            US100_Flag = 0;
            USART_SendData(USART3,0x55);//´¥·¢³¬Éù²¨
        }
        if(tim2flag>0)
        {
            Tim100ms++;
            tim2flag = 0;
            MPU6050_Pose();
            Getdesireddata();
            PID_CAL();

            Motor_Protect();
            Motor_PWM_OUTPUT(MOTOR1,MOTOR2,MOTOR3,MOTOR4);		//A6,A7,B0,B1

            Tim100ms = 0;
            CRC2[0]=(int)(Motor_Thr);
            CRC2[1]=(int)(Distance);
            CRC2[2]=(int)(thr_adder_Integration * 100);
            CRC2[3]=(int)(Channel_Receive[ROLL_D_BUS]);
            CRC_send(CRC2);
            

        }
    }
}
