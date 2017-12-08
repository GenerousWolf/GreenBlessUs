#include "stdafx.h"

/*state */


/* motor speed */
volatile VelocityWheel ActualSpeed;
volatile VelocityWheel ExpectSpeed;

/* info given */
volatile Point SelfPointArr[Max_Storage];
volatile Point BallPointArr[Max_Storage];
volatile Point RivalPointArr[Max_Storage];
const int GateX = 105;
const int GateY = 0;
volatile double courseAngle = 0.0;						//小车朝向,absolute angle
volatile uint16_t currentIndex = 0;							//current index in the array

/* strategy parameter*/
volatile Point TargetPoint;
volatile double TargetAngle;
volatile float VxbyDecision;
volatile float VybyDecision;
volatile float OmegabyDecision;

/* PID constant */
volatile float Kp = 0.1;
volatile float Ki = 0.05;
volatile float Kd = 0.1;
volatile float Umax = 200,Umin = -200;						//饱和上下限
volatile float Error_Max = 50;						//pid 单次最大误差，超过会导致积分项无效
/* PID struct */
volatile PIDstruct V0pid;
volatile PIDstruct V1pid;
volatile PIDstruct V2pid;

volatile u8 moveState;													//0 for stop, 1 for rotate, 2 for straightfoward
volatile s8 rotateStartIndex = 0; 							//index when starting rotate;
volatile uint8_t countNewPoint = 0;

volatile MatchInfo info;

void correctAngle(volatile double *uncorrectedAng)
{
	while(*uncorrectedAng > PI)
		*uncorrectedAng -= 2*PI;
	while(*uncorrectedAng < -PI)
		*uncorrectedAng += 2*PI;
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
	
	V0pid.ActualSpeed = -((s16)TIM2->CNT) / 1.25;
	V1pid.ActualSpeed = -((s16)TIM3->CNT) / 1.25;
	V2pid.ActualSpeed = -((s16)TIM4->CNT) / 1.25;
}

//TIM6 trigger a interupt 25hz
void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 3600 - 1; 		
    TIM_TimeBaseStructure.TIM_Period = 500; 				//40ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6, ENABLE);                    
}



double relaAngle(Point self, Point target)
{
	volatile int dy = target.Y - self.Y, dx = target.X - self.X;
	volatile float theta = (dy | dx) ? atan2(dy, dx) : 0.0f;
	return theta;
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
	float theta = courseAngle;
	ExpectSpeed.v0 = VxbyDecision * sin(theta - 2.094) - VybyDecision * cos(theta - 2.094) + OmegabyDecision;
	ExpectSpeed.v1 = VxbyDecision * sin(theta)		   - VybyDecision * cos(theta)		   + OmegabyDecision;
	ExpectSpeed.v2 = VxbyDecision * sin(theta + 2.094) - VybyDecision * cos(theta + 2.094) + OmegabyDecision;
	
	float max = (ExpectSpeed.v0 > ExpectSpeed.v1)? ExpectSpeed.v0:ExpectSpeed.v1;
	max = (max > ExpectSpeed.v2)?max:ExpectSpeed.v2;
	
	if(max > 80.0)
	{
		ExpectSpeed.v0 = ExpectSpeed.v0/max*80;
		ExpectSpeed.v1 = ExpectSpeed.v1/max*80;
		ExpectSpeed.v2 = ExpectSpeed.v2/max*80;
	}
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
	V0pid.SetSpeed = ExpectSpeed.v0+Kp*V0pid.err+flag * Ki*V0pid.integral+Kd*(V0pid.err-V0pid.err_last);
	V0pid.err_last = V0pid.err;
	
	flag = 1;
	
	V1pid.ExpectSpeed = ExpectSpeed.v1;
	V1pid.err = V1pid.ExpectSpeed - ActualSpeed.v1;
	if(fabs(V1pid.err) < Error_Max) flag = 0;
	if((V1pid.integral < Umax && V1pid.integral > Umin) ||
		 (V1pid.integral > Umax && V1pid.err < 0) ||
		 (V1pid.integral < Umin && V1pid.err > 0))	
		V1pid.integral += V1pid.err;
	V1pid.SetSpeed = ExpectSpeed.v1+Kp*V1pid.err+flag * Ki*V1pid.integral+Kd*(V1pid.err-V1pid.err_last);
	V1pid.err_last = V1pid.err;
	
	
	flag = 1;	
	
	V2pid.ExpectSpeed = ExpectSpeed.v2;
	V2pid.err = V2pid.ExpectSpeed - ActualSpeed.v2;
	if(fabs(V2pid.err) < Error_Max) flag = 0;
	if((V2pid.integral < Umax && V2pid.integral > Umin) ||
		 (V2pid.integral > Umax && V2pid.err < 0) ||
		 (V2pid.integral < Umin && V2pid.err > 0))
		V2pid.integral += V2pid.err;
	V2pid.SetSpeed = ExpectSpeed.v2+Kp*V2pid.err+flag * Ki*V2pid.integral+Kd*(V2pid.err-V2pid.err_last);
	V2pid.err_last = V2pid.err;
	
	/*
	V0pid.SetSpeed = ExpectSpeed.v0;
	V1pid.SetSpeed = ExpectSpeed.v1;
	V2pid.SetSpeed = ExpectSpeed.v2;
	*/
}

void Move(void)
{
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
	double diff = (p1.X-p2.X)*(p1.X-p2.X)+(p1.Y-p2.Y)*(p1.Y-p2.Y);
	return sqrt(diff);
}

void setStrategy(void)
{	
	double dDisSelfBall = getDistance(info.ptSelf, info.ptBall);
	double dDisRivalBall = getDistance(info.ptRival, info.ptBall);
	double dDisSelfRival = getDistance(info.ptSelf, info.ptRival);
	double dAngSelf = courseAngle;
	double dAngBallSelf = relaAngle(info.ptSelf, info.ptBall);
	double dAngRivalSelf = relaAngle(info.ptSelf, info.ptRival);
	
	if(info.nSelfState == ATK)
	{
		if(IsBoundary())
		{	
			TargetPoint.X = 105;
			TargetPoint.Y = 148;
			TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
			correctAngle(&TargetAngle);
			putchar('b');
			putchar('\n');
		}
		//Attack
		else if(!isBallDetected())
		{
			//Seek Ball
			TargetPoint = info.ptBall;
			TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
			correctAngle(&TargetAngle);
			putchar('s');
			putchar('\n');
		}
		else
		{
			//How to get close to gate
			if( info.ptSelf.Y < 30 &&
				info.ptSelf.X < 145 &&
				info.ptSelf.X > 65 )
			{
				//Shoot
				TargetPoint.X = 105;
				TargetPoint.Y = 5;
				TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
				correctAngle(&TargetAngle);
				putchar('a');
				putchar('\n');
			}
			else
			{
				if( (info.ptSelf.Y < info.ptRival.Y  && 
					info.ptSelf.Y >= 55 ) ||
					dDisSelfRival > 25 )
				{
					//Ready to Shoot
					TargetPoint.X = 105;
					TargetPoint.Y = 25;
					TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
					correctAngle(&TargetAngle);
					putchar('p');
					putchar('\n');
				}
				else if (dAngRivalSelf < 0.0f && dAngRivalSelf > - PI / 2)
				{
					TargetPoint.X = info.ptRival.X - 25 * sin(fabs(dAngRivalSelf));
					TargetPoint.Y = info.ptRival.Y - 25 * cos(fabs(dAngRivalSelf));
					TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
					correctAngle(&TargetAngle);
					putchar('l');
					putchar('\n');
				}
				else
				{
					TargetPoint.X = info.ptRival.X + 25 * sin(fabs(dAngRivalSelf));
					TargetPoint.Y = info.ptRival.Y + 25 * cos(fabs(dAngRivalSelf));
					TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
					correctAngle(&TargetAngle);
					putchar('r');
					putchar('\n');
				}
			}
		}
	}
	else
	{
		//Defend
		if(Visible)
		{
			//Ball Is Visible
			if(dDisRivalBall < 15)
			{
				TargetPoint = info.ptRival;
				TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
				correctAngle(&TargetAngle);
				putchar('c');
				putchar('\n');
			}
			else
			{
				if(isBallDetected())
				{
					TargetPoint.X = 150;
					TargetPoint.Y = 200;
					TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
					correctAngle(&TargetAngle);
					putchar('e');
					putchar('\n');
				}
				else
				{
					TargetPoint = info.ptBall;
					TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
					correctAngle(&TargetAngle);
					putchar('s');
					putchar('\n');
				}
			}
		}
		else
		{
			TargetPoint.X = (info.ptRival.X + 105) / 2;
			TargetPoint.Y = (info.ptRival.Y + 5) / 2;
			TargetAngle = relaAngle(info.ptSelf, TargetPoint) - dAngSelf;
			correctAngle(&TargetAngle);
			putchar('h');
			putchar('\n');
		}
	}
}

void DecideMove(void)
{
	int dx = TargetPoint.X - SelfPointArr[currentIndex].X;
	int dy = TargetPoint.Y - SelfPointArr[currentIndex].Y;
	int temp = sqrt((double)(dx*dx+dy*dy));
	int kOmega = 1;
	if(TargetAngle < 0)
		kOmega = -1;
	if(temp > 40.0){
		VxbyDecision = 150 * dx / temp;
		VybyDecision = 150 * dy / temp;
	}
	else{
		VxbyDecision = 3 * (temp + 10) * dx / temp;
		VybyDecision = 3 * (temp + 10) * dy / temp;
	}
	if(fabs(TargetAngle) > 2.5){//大于度，先旋转
		VxbyDecision = 0;
		VybyDecision = 0;
		OmegabyDecision = 40;
	}
	else if(fabs(TargetAngle) > 1.5f){//大于90度，先旋转
		VxbyDecision = 0;
		VybyDecision = 0;
		OmegabyDecision = 40 * kOmega;
	}
	else if(fabs(TargetAngle) > 0.5236f){//大概30度
		OmegabyDecision = 30 * kOmega;
	}
	else if(fabs(TargetAngle) > 0.1396f){//大概8度
		OmegabyDecision = 10 * kOmega;
	}
	else//小于8度
		OmegabyDecision = 0;
	if(info.nSelfState == DEF){
		if(fabs(VxbyDecision) > 30 ||
			fabs(VybyDecision) > 30){
				float max = (VxbyDecision > VybyDecision)?VxbyDecision:VybyDecision;
				VxbyDecision *= 30/max;
				VybyDecision *= 30/max;
		}
	}
	
}

int IsBoundary(void)
{
	int nRes = 0;
	if(	info.ptSelf.X >= 195 || 
		info.ptSelf.X <= 15  ||
		info.ptSelf.Y >= 282 )
	{
		if(info.ptSelf.X > 65 && info.ptSelf.X < 145)
		{
			nRes = 0;
		}
		else
		{
			nRes = 1;
		}
	}
	return nRes;
		
}
