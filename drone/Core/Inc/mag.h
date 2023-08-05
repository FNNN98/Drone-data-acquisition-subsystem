/*
 * mag.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef INC_MAG_H_
#define INC_MAG_H_

#include "bool.h"
#include "flight_dynamics.h"
#include "comm.h"
#include "dev.h"
#include "sensor_align.h"

typedef enum {
    MAG_DEFAULT = 0,
    MAG_NONE = 1,
    MAG_HMC5883 = 2,
    MAG_AK8975 = 3,
    MAG_AK8963 = 4,
    MAG_QMC5883 = 5,
    MAG_LIS3MDL = 6,
    MAG_MPU925X_AK8963 = 7
} magSensor_e;

typedef struct mag_s {
    float magADC[XYZ_AXIS_COUNT];
} mag_t;

struct magDev_s;

typedef void (*sensorMagInitFuncPtr)(struct magDev_s *magdev);
typedef bool (*sensorMagReadFuncPtr)(struct magDev_s *magdev);

typedef struct magDev_s {
	float magADC[XYZ_AXIS_COUNT];
    sensorMagInitFuncPtr init;                              // initialize function
    sensorMagReadFuncPtr read;                              // read 3 axis data function
    extiCallbackRec_t exti;
    extDevice_t dev;
    busDevice_t bus; // For MPU slave bus instance
    sensor_align_e magAlignment;
    fp_rotationMatrix_t rotationMatrix;
    ioTag_t magIntExtiTag;
    int16_t magGain[3];
} magDev_t;

extern mag_t mag;
extern magDev_t magDev;

void magInit(sensorMagInitFuncPtr, sensorMagReadFuncPtr);
void magUpdate(void);

#endif /* INC_MAG_H_ */
