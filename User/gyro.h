#ifndef _GYRO_H
#define _GYRO_H

#define TwoG (9.80f * 2.0f) 
#define MaxAngle (2000.0f)

extern volatile float AngRoll, AngPitch, AngYaw;
extern volatile float aX, aY, aZ, wX, wY, wZ;
extern volatile float ax0, ay0, az0, wx0, wy0, wz0;
void GetAngles(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z);
void SetupAllPivot(void);
void GetAllFromMPU(void);

#endif
