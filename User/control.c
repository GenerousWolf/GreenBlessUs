#include "stdafx.h"

/* motor speed */
volatile VelocityWheel ActualSpeed;
volatile VelocityWheel ExpectSpeed;

/* info given */
volatile Point SelfPointArr[Max_Storage];
volatile Point BallPointArr[Max_Storage];
volatile Point RivalPointArr[Max_Storage];
volatile float courseAngle = 0.450f;						//小车朝向,absolute angle
volatile uint16_t currentIndex = 0;							//current index in the array

/* strategy parameter*/
volatile Point TargetPoint;
volatile float TargetAngle;
volatile float VxbyDecision;
volatile float VybyDecision;
volatile float OmegabyDecision;

/* PID constant */
volatile float Kp = 0.2;
volatile float Ki = 0.05;
volatile float Kd = 0.1;
volatile float Umax = 200,Umin = 200;						//饱和上下限
volatile float Error_Max = 100;						//pid 单次最大误差，超过会导致积分项无效
/* PID struct */
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
	return uncorrectedAng;
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
	V0pid.ActualSpeed = ((s16)TIM2->CNT) - 12.0;
	V1pid.ActualSpeed = ((s16)TIM3->CNT) - 12.0;
	V2pid.ActualSpeed = ((s16)TIM4->CNT) - 12.0;
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


float relaAngle(Point self, Point target)
{
	volatile float theta = atan2(target.Y - self.Y, target.X - self.X);
	//printf("theta = %f\n", theta);
	return theta;
}
/*
int moveAngle(Point current, Point prev){
	return relaAngle(prev, current);
}
*/
void Stop(void)
{
	moveState = 0;
	Motor_Speed_Control(0,0);
	Motor_Speed_Control(0,1);
	Motor_Speed_Control(0,2);
}


//2.094 is the rad of 120°
void velocityConvert(void){
	float theta = courseAngle;
	ExpectSpeed.v0 = VxbyDecision * sin(theta - 2.094) - VybyDecision * cos(theta - 2.094) + OmegabyDecision;
	ExpectSpeed.v1 = VxbyDecision * sin(theta)				 - VybyDecision * cos(theta)				 + OmegabyDecision;
	ExpectSpeed.v2 = VxbyDecision * sin(theta + 2.094) - VybyDecision * cos(theta + 2.094) + OmegabyDecision;
}


void PIDcontrol(void)
{
	int flag = 1;
	
	V0pid.ExpectSpeed = ExpectSpeed.v0;
	V0pid.err = V0pid.ExpectSpeed - ActualSpeed.v0;
	if(fabs(V0pid.err) < Error_Max) flag = 0;											//积分分离
	if((V0pid.integral < Umax && V0pid.integral > Umin) ||
		 (V0pid.integral > Umax && V0pid.err < 0) ||
		 (V0pid.integral < Umin && V0pid.err > 0))
		V0pid.integral += V0pid.err;																//抗积分饱和
	V0pid.SetSpeed = Kp*V0pid.err+flag * Ki*V0pid.integral+Kd*(V0pid.err-V0pid.err_last);
	V0pid.err_last = V0pid.err;
	
	flag = 1;
	
	V1pid.ExpectSpeed = ExpectSpeed.v1;
	V1pid.err = V1pid.ExpectSpeed - ActualSpeed.v1;
	if(fabs(V1pid.err) < Error_Max) flag = 0;
	if((V1pid.integral < Umax && V1pid.integral > Umin) ||
		 (V1pid.integral > Umax && V1pid.err < 0) ||
		 (V1pid.integral < Umin && V1pid.err > 0))	
		V1pid.integral += V1pid.err;
	V1pid.SetSpeed = Kp*V1pid.err+flag * Ki*V1pid.integral+Kd*(V1pid.err-V1pid.err_last);
	V1pid.err_last = V1pid.err;
	
	
	flag = 1;	
	
	V2pid.ExpectSpeed = ExpectSpeed.v2;
	V2pid.err = V2pid.ExpectSpeed - ActualSpeed.v2;
	if(fabs(V2pid.err) < Error_Max) flag = 0;
	if((V2pid.integral < Umax && V2pid.integral > Umin) ||
		 (V2pid.integral > Umax && V2pid.err < 0) ||
		 (V2pid.integral < Umin && V2pid.err > 0))
		V2pid.integral += V2pid.err;
	V2pid.SetSpeed = Kp*V2pid.err+flag * Ki*V2pid.integral+Kd*(V2pid.err-V2pid.err_last);
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
	//printf("m0 = %d, m1 = %d, m2 = %d\n", (s16)V0pid.SetSpeed, (s16)V1pid.SetSpeed, (s16)V2pid.SetSpeed);
	Motor_Speed_Control((s16)V0pid.SetSpeed,0);
	Motor_Speed_Control((s16)V1pid.SetSpeed,1);
	Motor_Speed_Control((s16)V2pid.SetSpeed,2);
}


void MotorControl(void)
{
	motorspeedread();
	Encoder_Reset();
	
	setStrategy();
	DecideMove();
	
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
}


/******************************************************/
float getDistance(Point p1, Point p2)
{
	double dif = (p1.X-p2.X)*(p1.X-p2.X)+(p1.Y-p2.Y)*(p1.Y-p2.Y);
	return sqrt(dif);
}

void setStrategy(void)
{	
	double selfDistance = getDistance(SelfPointArr[currentIndex], BallPointArr[currentIndex]);
	double rivalDistance = getDistance(RivalPointArr[currentIndex], BallPointArr[currentIndex]);
	TargetPoint.X = BallPointArr[currentIndex].X;
	TargetPoint.Y = BallPointArr[currentIndex].Y;
	if( 0 /*selfDistance > rivalDistance*/){						//防守时位于球和本方球门之间；进攻时需要考虑球门的位置，使球门，球，车在一条直线，angle也需要更改
		TargetPoint.Y -= 12;
		if(TargetPoint.Y < 12)
			TargetPoint.Y = 12;
		TargetAngle = relaAngle(SelfPointArr[currentIndex], TargetPoint) - courseAngle;
	}
	else if(selfDistance > 15){
		TargetPoint = BallPointArr[currentIndex];
		TargetAngle = relaAngle(SelfPointArr[currentIndex], TargetPoint) - courseAngle;
	}
	else {//进入控球模式
		Point goalPoint;//球门坐标（105,297）
		goalPoint.X = 105;
		goalPoint.Y = 297;
		int dxBall2Goal = 105 - BallPointArr[currentIndex].X;
		int dyBall2Goal = 297 - BallPointArr[currentIndex].Y;
		double distanceBall2Goal = sqrt((double)(dxBall2Goal*dxBall2Goal + dyBall2Goal*dyBall2Goal));
		float angleBall2Goal = relaAngle(BallPointArr[currentIndex], goalPoint);
		TargetAngle = angleBall2Goal- courseAngle;
		if(fabs(angleBall2Goal - courseAngle) > 0.25){//调整姿态
			int dxBall2Goal = 105 - BallPointArr[currentIndex].X;
			int dyBall2Goal = 297 - BallPointArr[currentIndex].Y;
			double distanceBall2Goal = sqrt((double)(dxBall2Goal*dxBall2Goal + dyBall2Goal*dyBall2Goal));
			TargetPoint.X = BallPointArr[currentIndex].X - selfDistance * dxBall2Goal / distanceBall2Goal;
			TargetPoint.X = BallPointArr[currentIndex].Y - selfDistance * dyBall2Goal / distanceBall2Goal;
		}
		else{//推球前进，准备射门
			TargetPoint = goalPoint;
		}
	}
}

void DecideMove(void)
{
	int dx = BallPointArr[currentIndex].X - SelfPointArr[currentIndex].X;
	int dy = BallPointArr[currentIndex].Y - SelfPointArr[currentIndex].Y;
	int temp = sqrt((double)(dx*dx+dy*dy));
	int kOmega = 1;
	if(TargetAngle < 0)
		kOmega = -1;
	VxbyDecision = 200 * dx / temp;
	VybyDecision = 200 * dy / temp;
	if(fabs(TargetAngle) > 1.5){//大于90度，先旋转
		VxbyDecision = 0;
		VybyDecision = 0;
	}
	else if(fabs(TargetAngle) > 0.5){//大概30度
		OmegabyDecision = 200 * kOmega;
	}
	else if(fabs(TargetAngle) > 0.25){//大概15度
		OmegabyDecision = 80 * kOmega;
	}
	else//小于15度
		OmegabyDecision = 0;
}
