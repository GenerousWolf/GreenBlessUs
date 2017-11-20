#ifndef __STDAFX_H
#define __STDAFX_H

// All Dependencies

/**Runtime**/
#include <stm32f10x.h>
#include <stm32f10x_i2c.h>
#include "stm32f10x_it.h"
#include <misc.h>

/**Standard C Libraries**/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/**Users' Headers**/
#include "gyro.h"
#include "control.h"
#include "usart.h"
#include "nvic.h"

/**Private Headers**/
#include "motor.h"
#include "mpu6050.h"
#include "systick.h"

#endif
