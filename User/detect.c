#include "stdafx.h"

void Detect_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

int isBallDetected(void)
{
	int result = 0;
	if(GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_0) == 0)
		result++;
	if(GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_1) == 0)
		result++;
	if(GPIO_ReadInputDataBit (GPIOC,GPIO_Pin_2) == 0)
		result++;
	return result;
}

int isToWall(void)
{
	if(SelfPointArr[currentIndex].X < 15
		&& ((fabs(courseAngle - PI) < PI/8) || (fabs(courseAngle + PI) < 0.4)))//45業
		return 1;
	if(SelfPointArr[currentIndex].X > 195
		&& fabs(courseAngle) < PI/8)//45業
		return 1;
	if(SelfPointArr[currentIndex].Y < 15
		&& fabs(courseAngle - PI/2) < PI/8)//45業
		return 1;
	if(SelfPointArr[currentIndex].Y > 275
		&& fabs(courseAngle + PI/2) < PI/8)//45業
		return 1;
	return 0;
}
