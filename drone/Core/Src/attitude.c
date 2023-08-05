/*
 * attitude.c
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#include "attitude.h"
#include "imu.h"

void attitudeInit(void) {
	imuInit();
}

void attitudeUpdate(void)
{
	imuCalculateEstimatedAttitude();
	imuCalculateThrottleAngleCorrection();
}
