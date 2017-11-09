#include "stdafx.h"

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 0.005f                   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0000f                	// integral gain governs rate of convergence of gyroscope biases
#define halfT 0.01f                	// half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float q0 = 1, q1 = 0, q2 = 0, q3 = 0;        // quaternion elements representing the estimated orientation
volatile float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
volatile float aX, aY, aZ, wX, wY, wZ;
volatile float ax0, ay0, az0, wx0, wy0, wz0;
volatile float AngRoll, AngPitch, AngYaw;
volatile float norm;
volatile float vx, vy, vz;
volatile float ex, ey, ez; 
//====================================================================================================
// Function
//====================================================================================================

void GetAngles(float ax, float ay, float az ,float gx, float gy, float gz) 
{
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);      
	ax/=norm;
	ay/=norm;
	az/=norm;      

	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	// integral error scaled integral gain
	exInt+=ex*Ki*0.001;
	eyInt+=ey*Ki*0.001;
	ezInt+=ez*Ki*0.001;

	// adjusted gyroscope measurements
	gx+=Kp*ex + exInt;
	gy+=Kp*ey + eyInt;
	gz+=Kp*ez + ezInt;

	// integrate quaternion rate and normalise
	q0+=(-q1*gx - q2*gy - q3*gz)*halfT;
	q1+=(q0*gx + q2*gz - q3*gy)*halfT;
	q2+=(q0*gy - q1*gz + q3*gx)*halfT;
	q3+=(q0*gz + q1*gy - q2*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0/=norm;
	q1/=norm;
	q2/=norm;
	q3/=norm;
	
	AngYaw = atan2(2.0*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}

void GetAllFromMPU(void)
{
	aX = GetData(ACCEL_XOUT_H) / 32768.0f * TwoG - ax0;
	aY = GetData(ACCEL_YOUT_H) / 32768.0f * TwoG - ay0;
	aZ = GetData(ACCEL_ZOUT_H) / 32768.0f * TwoG - az0;
	wX = GetData(GYRO_XOUT_H) / 32768.0f * MaxAngle - wx0;
	wY = GetData(GYRO_YOUT_H) / 32768.0f * MaxAngle - wy0;
	wZ = GetData(GYRO_ZOUT_H) / 32768.0f * MaxAngle - wz0;
	wX = wX / 180.0f * 3.14f;
	wY = wY / 180.0f * 3.14f;
	wZ = wZ / 180.0f * 3.14f;
	GetAngles(aX,aY,aZ,wX,wY,wZ);
}

void SetupAllPivot(void)
{
	for(int i = 0; i < 1000; ++i)
	{
		ax0 += GetData(ACCEL_XOUT_H) / 32768.0f * TwoG;
		ay0 += GetData(ACCEL_YOUT_H) / 32768.0f * TwoG;
		az0 += GetData(ACCEL_ZOUT_H) / 32768.0f * TwoG;
		wx0 += GetData(GYRO_XOUT_H) / 32768.0f * MaxAngle;
		wy0 += GetData(GYRO_YOUT_H) / 32768.0f * MaxAngle;
		wz0 += GetData(GYRO_ZOUT_H) / 32768.0f * MaxAngle;
	}
	ax0 /= 1000;
	ay0 /= 1000;
	az0 /= 1000;
	wx0 /= 1000;
	wy0 /= 1000;
	wz0 /= 1000;
}

