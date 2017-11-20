#include "stdafx.h"

volatile s16 motorSpeedMeas[3];
volatile float_t veloVehi[3];
volatile Point SelfPointArr[Max_Storage];
volatile Point BallPointArr[Max_Storage];
volatile s16 SelfAngleArr[Max_Storage];				//小车朝向,absolute angle
volatile s16 TargetAngleArr[Max_Storage];			//球相对小车的角度,absolute angle
volatile u8 moveState;				//0 for stop, 1 for rotate, 2 for straightfoward
volatile s8 currentIndex = 0;									//current index in the array
volatile s8 rotateStartIndex = 0; 							//index when starting rotate;
volatile uint8_t countNewPoint = 0;

volatile MatchInfo info;

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
			count = -314;
			break;
	}
	return (s16)count;
}

void motorspeedread(void)
{
	motorSpeedMeas[0] = TIM2 -> CNT;
	motorSpeedMeas[1] = TIM3 -> CNT;
	motorSpeedMeas[2] = TIM4 -> CNT;
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


int relaAngle(Point self, Point target){
	double theta = atan2(target.Y - self.Y, target.X - self.X);
	return (s16) (100*theta);
}

int moveAngle(Point current, Point prev){
	return relaAngle(prev, current);
}

void getSelfAngle(void)
{
	int angle = (int)(-112 * AngYaw);
	if(angle > 314) {
		angle += 314;
		angle %= 628;
		angle -= 314;
	}
	else
		if(angle < -314) {
			angle *= -1;
			angle += 314;
			angle %= 628;
			angle -= 314;
			angle *= -1;
		}
	SelfAngleArr[currentIndex] = angle;
}	

void Stop(void)
{
	moveState = 0;
	Motor_Speed_Control(0,0);
	Motor_Speed_Control(0,1);
	Motor_Speed_Control(0,2);
}

void Move(void)
{
	s16 relativeAngle = TargetAngleArr[currentIndex] - SelfAngleArr[currentIndex];
	if(relativeAngle > 314) 
	{
		relativeAngle += 314;
		relativeAngle %= 628;
		relativeAngle -= 314;
	}
	else if(relativeAngle < -314) 
	{
		relativeAngle *= -1;
		relativeAngle += 314;
		relativeAngle %= 628;
		relativeAngle -= 314;
		relativeAngle *= -1;
	}		
	Rotate(relativeAngle);
	printf("RelaAngle = %d\n", relativeAngle);
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
