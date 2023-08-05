/*
 * sensors.h
 *
 *  Created on: Jun 10, 2022
 *      Author: pietro
 */

#ifndef SRC_SENSORS_H_
#define SRC_SENSORS_H_

#include "accel.h"
#include "baro.h"
#include "gyro.h"
#include "mag.h"

void sensorsInit(sensorGyroInitFuncPtr, sensorGyroReadFuncPtr, sensorAccInitFuncPtr, sensorAccReadFuncPtr, sensorMagInitFuncPtr, sensorMagReadFuncPtr, sensorBaroInitFuncPtr, sensorBaroReadFuncPtr);
void sensorsUpdate(void);

#endif /* SRC_SENSORS_H_ */
