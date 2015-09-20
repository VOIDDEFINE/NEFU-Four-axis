#include "control.h"
#include <math.h>
#include "delay.h"
#include "mpu6050.h"
#include "include.h" 

pidsuite pidRoll;					   //�����pid
pidsuite pidPitch;					 //������pid
pidsuite pidYaw;
pidsuite pidThr;

float Hode_temp[3] = {0};

int Motor_Thr=0;					   //����
int Motor_Ele=0;					   //��������
int Motor_Ail=0;					   //�������
int Motor_Rud=0;					   //��������
float thr_adder = 325;
float thr_adder_Integration = 0;
int MOTOR1;
int MOTOR2;
int MOTOR3;
int MOTOR4;

PID_Out pid_out;

extern float Pitch,Roll,Yaw;

float Gyro_FinalX=0;
float Gyro_FinalY=0;
float Gyro_FinalZ=0;
float LastGyro_FinalX=0;
float LastGyro_FinalY=0;
float LastGyro_FinalZ=0;

int pwmout1,pwmout2,pwmout3,pwmout4; 				//���ռ�ձ�
/*------------------------------------------pid�ṹ��ʼ��-----------------------------------------*/
//����������ṹ��ָ�룬����ֵ��kp,ki,kd
void pidInit(pidsuite* pid, const float desired, const float kp,
             const float ki, const float kd)
{

    pid->error = 0;
    pid->prevError = 0;
    pid->integ = 0;
    pid->deriv = 0;
    pid->desired = desired;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}

/*----------------------------------------------pid�������------------------------------------------*/
//���������pid�ṹ��ָ�룬����ֵ ,����ֵ
//�����pid���
float pidUpdate(pidsuite* pid, const float measured,float expect,float gyro)
{
    float output;
    static float lastoutput=0;

    pid->desired=expect;			 				//��ȡ�����Ƕ�

    pid->error = pid->desired - measured;	 	  //ƫ�����-����ֵ

    pid->integ += pid->error * IMU_UPDATE_DT;	  //ƫ�����

    if (pid->integ > pid->iLimit)				  //����������
    {
        pid->integ = pid->iLimit;
    }
    else if (pid->integ < -pid->iLimit)
    {
        pid->integ = -pid->iLimit;
    }

    //pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//΢��	 Ӧ�ÿ��������ǽ��ٶȴ���
    pid->deriv = -gyro;
    if(fabs(pid->error)>Piddeadband)									//pid����
    {
        pid->outP = pid->kp * pid->error;								 //��������۲�
        pid->outI = pid->ki * pid->integ;
        pid->outD = pid->kd * pid->deriv;

        output = (pid->kp * pid->error) +
                 (pid->ki * pid->integ) +
                 (pid->kd * pid->deriv);
    }
    else
    {
        output=lastoutput;
    }

    pid->prevError = pid->error;							 		//����ǰһ��ƫ��
    lastoutput=output;

    return output;
}

/*----------------------------------------------pid���ֲ�������ֵ-------------------------------------------*/
void pidSetIntegralLimit(pidsuite* pid, const float limit)
{
    pid->iLimit = limit;
}


void PID_controllerInit(void)
{
    pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
    pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
    pidInit(&pidYaw,0,PID_YAW_KP,PID_YAW_KI,PID_YAW_KD);

    //pidInit(&pidThr,250,PID_DIS_KP,PID_DIS_KI,PID_DIS_KD);

    pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
    pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);

    //pidSetIntegralLimit(&pidThr, PID_DIS_INTEGRATION_LIMIT);
}


/*------------------------------------pid����------------------------------------*/
void PID_CAL(void)		  //PID��������
{
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    gyro_roll = gyro[0];
    gyro_pitch = gyro[1];
    gyro_yaw = gyro[2];

    //������xyz�����ϵ�ԭʼ������10����
    gyro_roll = gyro_roll / 10;
    gyro_pitch = gyro_pitch / 10;
    gyro_yaw = gyro_yaw / 10;

    pid_out.roll  = pidUpdate(&pidRoll,  Roll,  Motor_Ail, gyro_roll);
    pid_out.pitch = pidUpdate(&pidPitch, Pitch, Motor_Ele, gyro_pitch);
    pid_out.yaw   = pidUpdate(&pidYaw,   Yaw,   Motor_Rud, gyro_yaw);
    //pid_out.yaw = 0;(Channel_Receive[YAW_D_BUS] - 1024)/22

//	//ȥ���ջ����
//	pid_out.yaw = 0;
//	pid_out.roll = 0;
//	pid_out.pitch = 0;

    MOTOR1=MOTORLimit(Motor_Thr + pid_out.roll + pid_out.pitch + pid_out.yaw + 0);	//A6�ŵ��

    MOTOR2=MOTORLimit(Motor_Thr - pid_out.roll + pid_out.pitch - pid_out.yaw - 1);	//A7�ŵ��

    MOTOR3=MOTORLimit(Motor_Thr - pid_out.roll - pid_out.pitch + pid_out.yaw + 0);	//B0�ŵ��

    MOTOR4=MOTORLimit(Motor_Thr + pid_out.roll - pid_out.pitch - pid_out.yaw - 2);	//B1�ŵ��

    if(Motor_Thr<=255)
    {
        MOTOR1=250;
        MOTOR2=250;
        MOTOR3=250;
        MOTOR4=250;
    }
}

/*-----------------------------------------����������-------------------------------------------*/
int MOTORLimit(float value)
{
    if(value>500)
    {
        value=500;
    }
    else if(value<250)
    {
        value=250;
    }
    else
    {
        value=value;
    }

    return value;
}
uint8_t Holding_flag = 0;
float thr_temp;
extern int Distance;
int last_distance;
int Distance_holding;

float Thr_I_Up=30000;    //10000
float Thr_I_Down=90000;  //20000
float Thr_P_Up=0.008;     //0.03
float Thr_P_Down=0.002;   //0.01
float Thr_D_Up=0.01;
float Thr_D_Down=0.08;


void Getdesireddata(void)//ң��
{
    float pitch_temp;
    float roll_temp;
    float yaw_temp;

    //���û����Ż����ܵı���
    float deltaE;
    static uint8_t M;


    static float yaw_adder = 0;
    thr_temp = Channel_Receive[THR_D_BUS] - 1024;//��������
    pitch_temp = Channel_Receive[PITCH_D_BUS] - 1024;//��������
    roll_temp = Channel_Receive[ROLL_D_BUS] - 1024;//�������
    yaw_temp = Channel_Receive[YAW_D_BUS] - 1024;//��������

    if(thr_temp == 0 && Holding_flag == 0)//���ɿ�����
    {
        //Distance_holding = Distance;//��¼�ɿ�����ʱ��ĸ߶�
        Distance_holding = 500;//Ҫ����ߵĸ�ֵ
        Holding_flag = 1;
    }
    if(thr_temp != 0 && Holding_flag == 1)
    {
        Holding_flag = 0;
    }

    Hode_temp[0] = Distance_holding - Distance;//���ƫ��

    //�Զ����ߴ��뿪ʼ
    if(Holding_flag == 1 && Distance < 35000  && Distance > 30)
    {
        deltaE = Hode_temp[0] - Hode_temp[1];
        if( (Hode_temp[0]*deltaE>0) || (deltaE==0&&(Hode_temp[0])!=0) )
            M=1;
        else if(Hode_temp[0]*deltaE<0 || Hode_temp[0]==0)
            M=0;

        if(Hode_temp[0] > 0)//��������
        {
            Thr_I_Up = 50000 - (Hode_temp[0] * Hode_temp[0] * 0.10);
            if(Thr_I_Up < 20000) Thr_I_Up = 20000;
            if(Thr_I_Up > 50000) Thr_I_Up = 50000;

            //pid�����������̣��Ը߶ȱջ����ƣ�pid������ӵ�ң�����ۼ�������
            thr_adder_Integration += ( Hode_temp[0] / Thr_I_Up ) * M ;
            thr_adder = 390 + thr_adder_Integration + Hode_temp[0] * Thr_P_Up + (Hode_temp[0] - Hode_temp[1]) * Thr_D_Up;
        }
        else if (Hode_temp[0] < 0)//�½�����
        {
            Thr_I_Down = 90000 - (Hode_temp[0] * Hode_temp[0] * 1.1);
            if(Thr_I_Down < 80000) Thr_I_Down = 80000;
            if(Thr_I_Down > 90000) Thr_I_Down = 90000;

            //pid�����½����̣��Ը߶ȱջ����ƣ�pid������ӵ�ң�����ۼ�������
            thr_adder_Integration += ( Hode_temp[0] / Thr_I_Down )* M ;
            thr_adder = 390 + thr_adder_Integration + Hode_temp[0] * Thr_P_Down + (Hode_temp[0] - Hode_temp[1]) * Thr_D_Down;
        }
        //�Զ��߹��̣����ŵ�������
        if(thr_adder >=406)
        {
            thr_adder = 406;
        }
        if(thr_adder < 390)
        {
            thr_adder = 390;
        }
    }//�Զ����� �������

    last_distance = Distance;
    Hode_temp[2] = Hode_temp[1];
    Hode_temp[1] = Hode_temp[0];
    thr_adder += thr_temp/5000;

    //�ֶ�����ʱ��ң��������������������
    if(thr_adder > 420)
    {
        thr_adder = 420;
    }
    if(thr_adder < 250)
    {
        thr_adder = 250;
    }

    yaw_adder += yaw_temp/5000;
    if(yaw_adder > 100)
    {
        yaw_adder = 100;
    }
    if(yaw_adder < -100)
    {
        yaw_adder = -100;
    }
    Motor_Thr = (int)thr_adder;
    Motor_Ele = (int)(0 - pitch_temp/44);				//ת����λΪ�Ƕ�-15~15��
    Motor_Ail = (int)(0 - roll_temp/44);
    Motor_Rud = (int)(0 - yaw_adder);

    if(Motor_Rud<2&&Motor_Rud>-2)
    {
        Motor_Rud=0;
    }
    else
    {
        Motor_Rud=Motor_Rud;
    }				//-100~100��

}

void Motor_Protect(void)
{
    uint16_t Protect_flag_S1;
    uint16_t Protect_flag_S2;
    Protect_flag_S1 = Channel_Receive[S1];
    Protect_flag_S1 = Channel_Receive[S2];
    if(Protect_flag_S1 != 1 && Protect_flag_S2 != 1)
    {
        thr_adder = 325;
        MOTOR1 = 250;
        MOTOR2 = 250;
        MOTOR3 = 250;
        MOTOR4 = 250;
    }
}

