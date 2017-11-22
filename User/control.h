#ifndef CONTROL_H
#define CONTROL_H

#include "stdafx.h"

#define MOTOR0_TIM TIM2
#define MOTOR1_TIM TIM3
#define MOTOR2_TIM TIM4
#define Max_Storage (100)
#define PI (3.1415926535f)
#define RADIUS_VEHI (10)

typedef int16_t XType, YType;

typedef struct tagPOINT
{
	XType X;
	YType Y;
}Point;

enum SHOOTOUT {NO = 0x0, YES = 0x1};
enum MATCHSTATUS {NOTBEGIN = 0x0, RUNNING = 0x1, PAUSE = 0x2, OVER = 0x3};
enum SIDE {SIDE_SELF = 0x1, SIDE_RIVAL = 0x0};

typedef struct tagMATCHINFO
{
	uint16_t uTimeByRounds;
	uint16_t nHaltRoundsSelf;
	uint16_t nHaltRoundsRival;
	uint8_t byteShootOut;
	uint8_t byteShootSide;
	uint8_t byteMatchStatus;
	uint8_t nEvilSelf;
	uint8_t nEvilRival;
	uint8_t nScoreSelf;
	uint8_t nScoreRival;
	Point ptBall;
	Point ptSelf;
	Point ptRival;
}MatchInfo;

typedef struct WHEELVELOCITY
{
	float v0;
	float v1;
	float v2;
}VelocityWheel;

typedef struct _PID{
		float ExpectSpeed;					//期望值
    float SetSpeed;            	//最终的设定值
    float ActualSpeed;        	//测量所得的实际值
    float err;                	//偏差值
    float err_last;            	//上一个偏差值
    float Kp,Ki,Kd;            	//比例、积分、微分系数
    float integral;            	//积分值
}PIDstruct;


extern volatile s16 motorSpeedMeas[3];
extern volatile float_t veloVehi[3];
extern volatile Point SelfPointArr[Max_Storage];
extern volatile Point BallPointArr[Max_Storage];
extern volatile Point RivalPointArr[Max_Storage];
extern volatile float courseAngleArr[Max_Storage];				//小车朝向,absolute angle
extern volatile float TargetAngleArr[Max_Storage];				//球相对小车的角度,absolute angle
extern volatile s8 currentIndex;										//current index in the array
extern volatile float VxbyDecision;
extern volatile float VybyDecision;
extern volatile float OmegabyDecision;
extern volatile VelocityWheel ActualSpeed;
extern volatile VelocityWheel ExpectSpeed;
extern volatile PIDstruct V0pid;
extern volatile PIDstruct V1pid;
extern volatile PIDstruct V2pid;
extern volatile u8 moveState;													//0 for stop, 1 for rotate, 2 for straightfoward
extern volatile s8 rotateStartIndex; 							//index when starting rotate;
extern volatile uint8_t countNewPoint;
extern volatile MatchInfo info;


//correct course angle to -pi~pi
float correctAngle(float uncorrectedAng);

void Encoder_Init(void);
void Encoder_Reset(void);
int Encoder_Read(int motornum);
void motorspeedread(void);

void TIM6_Init(void);

void RotateStop(int16_t rotateAngle);
void Rotate(s16 angle);

float relaAngle(Point self, Point target);
//int moveAngle(Point current, Point prev);

void getSelfAngle(void);

//convert the overall velocity to the motor speed by a certain matrix
void velocityConvert(void);

void PID_init(void);
void PIDcontrol(void);
void move(void);
void Stop(void);
void MotorControl(void);
void addNewPoint(Point selfPoint, Point ballPoint);
#endif
