#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  100.0
#define IMU_UPDATE_FREQ   250		//250hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)		//积分常数

#define PID_ROLL_KP  0.7//0.4//0.46//0.4
#define PID_ROLL_KI  0.35//0//0.0 //2.0	 0.1//0.35
#define PID_ROLL_KD  0.115//0.15//0.1 //0.15


#define PID_ROLL_INTEGRATION_LIMIT    25.0 // 20.0

#define PID_PITCH_KP  0.7//0.4
#define PID_PITCH_KI  0.35//0.0		  0.1
#define PID_PITCH_KD  0.115//0.09
#define PID_PITCH_INTEGRATION_LIMIT   25.0 // 20.0

#define PID_YAW_KP 0.8					  //0.5~1.0			 2大了
#define PID_YAW_KI 0.01
#define PID_YAW_KD 0.5					   //参数前加负号
#define PID_YAW_INTEGRATION_LIMIT   20.0 // 20.0

#define PID_DIS_KP 0.01
#define PID_DIS_KI 0.01
#define PID_DIS_KD 0.00
#define PID_DIS_INTEGRATION_LIMIT 5

#define Piddeadband 0		  //1~2
extern float thr_temp;
extern int Distance_holding;
extern uint8_t Holding_flag;

extern int MOTOR1;
extern int MOTOR2;
extern int MOTOR3;
extern int MOTOR4;

extern int Motor_Thr;
extern int Motor_Ele;					   //俯仰期望
extern int Motor_Ail;					   //横滚期望
extern int Motor_Rud;					   //航向期望

extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;


typedef struct
{
    float desired;     //< 被调量期望值
    float error;        //< 期望值-实际值
    float prevError;    //< 前一次偏差
    float integ;        //< 积分部分
    float deriv;        //< 微分部分
    float kp;           //< 比例参数
    float ki;           //< 积分参数
    float kd;           //< 微分参数
    float outP;         //< pid比例部分，调试用
    float outI;         //< pid积分部分，调试用
    float outD;         //< pid微分部分，调试用
    float iLimit;      //< 积分限制
} pidsuite;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} PID_Out;

extern pidsuite pidRoll;
extern pidsuite pidPitch;
extern pidsuite pidYaw;
extern int pwmout1,pwmout2,pwmout3,pwmout4;

void pidInit(pidsuite* pid, const float desired, const float kp, const float ki, const float kd);
float pidUpdate(pidsuite* pid, const float measured,float expect,float gyro);
void pidSetIntegralLimit(pidsuite* pid, const float limit);
void PID_controllerInit(void);
void PID_CAL(void);
int MOTORLimit(float value);
void Getdesireddata(void);
void controlmiddleinit(void);
void Motor_Protect(void);
//float yawcontrol(float gz,const float measured,float expect);

#endif
