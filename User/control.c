#include "stdafx.h"

volatile s16 motorSpeedMeas[3];
volatile float_t veloVehi[3];

volatile Point SelfPointArr[Max_Storage];
volatile Point BallPointArr[Max_Storage];
volatile Point RivalPointArr[Max_Storage];

volatile float courseAngleArr[Max_Storage];				//小车朝向,absolute angle
volatile float TargetAngleArr[Max_Storage];				//球相对小车的角度,absolute angle

volatile s8 currentIndex = 0;										//current index in the array

volatile float VxbyDecision;
volatile float VybyDecision;
volatile float OmegabyDecision;

volatile VelocityWheel ActualSpeed;
volatile VelocityWheel ExpectSpeed;

volatile PIDstruct V0pid;
volatile PIDstruct V1pid;
volatile PIDstruct V2pid;

volatile u8 moveState;													//0 for stop, 1 for rotate, 2 for straightfoward
volatile s8 rotateStartIndex = 0; 							//index when starting rotate;
volatile uint8_t countNewPoint = 0;

volatile MatchInfo info;

float correctAngle(float uncorrectedAng)
{
	while(uncorrectedAng > PI)
		uncorrectedAng -= 2*PI;
	while(uncorrectedAng < -PI)
		uncorrectedAng += 2*PI;
}

void Encoder_Init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	//motor feedback input
	//motor0 - PA0,PA1(TIM2 CH1,CH2)
	//motor1 - PA6,PA7(TIM3 CH1,CH2)
	//motor2 - PB6,PB7(TIM4 CH1,CH2)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = 0xffff;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure); 
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
   
	//Reset counter
	TIM2 -> CNT = 0;
	TIM3 -> CNT = 0;
	TIM4 -> CNT = 0;
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

//计数寄存器赋值
void Encoder_Reset(void)
{
	TIM2 -> CNT = 0;
	TIM3 -> CNT = 0;
	TIM4 -> CNT = 0;
}

//incompleted
//寄存器值读取
int Encoder_Read(int motornum)
{
	int count;
	switch(motornum)
		{
		case 0:
			count = TIM2->CNT;
			break;
		case 1:
			count = TIM3->CNT;
			break;
		case 2:
			count = TIM4->CNT;
			break;
		default:
			count = -111;
			break;
	}
	return (s16)count;
}


//the parameter is to be measured
void motorspeedread(void)
{
	motorSpeedMeas[0] = TIM2 -> CNT;
	motorSpeedMeas[1] = TIM3 -> CNT;
	motorSpeedMeas[2] = TIM4 -> CNT;
	
	V0pid.ActualSpeed = motorSpeedMeas[0] / 11.0;
	V1pid.ActualSpeed = motorSpeedMeas[1] / 11.0;
	V2pid.ActualSpeed = motorSpeedMeas[2] / 11.0;
}

//TIM6 trigger a interupt per 20ms
void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 3600 - 1; 		//1kHz 1Tick = 1ms
    TIM_TimeBaseStructure.TIM_Period = 20 - 1; 				//20ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6, ENABLE);                    
}

//rotate is from -314 to 314
void RotateStop(int16_t rotateAngle){
	rotateAngle *= 11;
	rotateAngle /= 2;
	if((Encoder_Read(0) > rotateAngle || (-Encoder_Read(0)) > rotateAngle)){
			Motor_Speed_Control(0, 0);
			Motor_Speed_Control(0, 1);
			Motor_Speed_Control(0, 2);
			Encoder_Reset();
	}
}

void Rotate(s16 angle)
{
	moveState = 1;
	rotateStartIndex = currentIndex;
	if(angle > 157){					//方向角偏移超过60°,原地旋转
		Motor_Speed_Control(20,0);
		Motor_Speed_Control(20,1);
		Motor_Speed_Control(20,2);
	}
	else {
		if(angle < -157){
			Motor_Speed_Control(-20,0);
			Motor_Speed_Control(-20,1);
			Motor_Speed_Control(-20,2);
		}
		else{
			if(angle > 30)
			Motor_Speed_Control(0,0);
			Motor_Speed_Control(0,1);
			Motor_Speed_Control(0,2);
		}
	}
	Encoder_Reset();
}

void straightfoward()
{
	moveState = 2;
	Motor_Speed_Control(-50,0);
	Motor_Speed_Control(50,2);
	Motor_Speed_Control(0,1);
}


float relaAngle(Point self, Point target){
	float theta = atan2(target.Y - self.Y, target.X - self.X);
	return theta;
}
/*
int moveAngle(Point current, Point prev){
	return relaAngle(prev, current);
}
*/
void getSelfAngle(void)
{
	float angle = -1.12 * AngYaw;
	courseAngleArr[currentIndex] = angle;
}

void Stop(void)
{
	moveState = 0;
	Motor_Speed_Control(0,0);
	Motor_Speed_Control(0,1);
	Motor_Speed_Control(0,2);
}


//2.094 is the rad of 120°
void velocityConvert(void){
	float theta = courseAngleArr[currentIndex];
	ExpectSpeed.v0 = VxbyDecision * sin(theta - 2.094) - VybyDecision * cos(theta - 2.094) + OmegabyDecision;
	ExpectSpeed.v1 = VxbyDecision * sin(theta)				 - VybyDecision * cos(theta)				 + OmegabyDecision;
	ExpectSpeed.v2 = VxbyDecision * sin(theta + 2.094) - VybyDecision * cos(theta + 2.094) + OmegabyDecision;
}


void PID_init(void)
{
	V0pid.Kp=0.2;
  V0pid.Ki=0.015;
  V0pid.Kd=0.2;
	
	V1pid.Kp=0.2;
  V1pid.Ki=0.015;
  V1pid.Kd=0.2;
	
	V2pid.Kp=0.2;
  V2pid.Ki=0.015;
  V2pid.Kd=0.2;
}

void PIDcontrol(void)
{
	V0pid.ExpectSpeed = ExpectSpeed.v0;
	V1pid.ExpectSpeed = ExpectSpeed.v1;
	V2pid.ExpectSpeed = ExpectSpeed.v2;
	
	V0pid.err = V0pid.ExpectSpeed - ActualSpeed.v0;
	V1pid.err = V0pid.ExpectSpeed - ActualSpeed.v1;
	V2pid.err = V0pid.ExpectSpeed - ActualSpeed.v2;
	
	V0pid.integral += V0pid.err;
	V1pid.integral += V1pid.err;
	V2pid.integral += V2pid.err;
	
	V0pid.SetSpeed = V0pid.Kp*V0pid.err+V0pid.Ki*V0pid.integral+V0pid.Kd*(V0pid.err-V0pid.err_last);
	V1pid.SetSpeed = V1pid.Kp*V1pid.err+V1pid.Ki*V1pid.integral+V1pid.Kd*(V1pid.err-V1pid.err_last);
	V2pid.SetSpeed = V2pid.Kp*V2pid.err+V2pid.Ki*V2pid.integral+V2pid.Kd*(V2pid.err-V2pid.err_last);
	
	V0pid.err_last = V0pid.err;
	V1pid.err_last = V1pid.err;
	V2pid.err_last = V2pid.err;
}

void Move(void)
{
	float max = (V0pid.SetSpeed > V1pid.SetSpeed)? V0pid.SetSpeed:V1pid.SetSpeed;
	max = (max > V2pid.SetSpeed)?max:V2pid.SetSpeed;
	
	if(max > 100.0)
	{
		V0pid.SetSpeed = V0pid.SetSpeed/max*100;
		V1pid.SetSpeed = V1pid.SetSpeed/max*100;
		V2pid.SetSpeed = V2pid.SetSpeed/max*100;
	}
	
	Motor_Speed_Control((s16)V0pid.SetSpeed,0);
	Motor_Speed_Control((s16)V1pid.SetSpeed,1);
	Motor_Speed_Control((s16)V2pid.SetSpeed,2);
}


void MotorControl(void)
{
	motorspeedread();
	Encoder_Reset();
	velocityConvert();
	PIDcontrol();
	Move();
}


void addNewPoint(Point selfPoint, Point ballPoint)
{
	currentIndex++;
	currentIndex %= Max_Storage;
	SelfPointArr[currentIndex] = selfPoint;
	BallPointArr[currentIndex] = ballPoint;
	TargetAngleArr[currentIndex] = relaAngle(info.ptSelf, info.ptBall);
	countNewPoint++;
}
