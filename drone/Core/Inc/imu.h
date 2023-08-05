/*
 * imu.h
 *
 *  Created on: Jun 10, 2022
 *      Author: pietro
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

void imuInit(void);
void imuCalculateEstimatedAttitude(void);
void imuCalculateThrottleAngleCorrection(void);

#endif /* INC_IMU_H_ */
