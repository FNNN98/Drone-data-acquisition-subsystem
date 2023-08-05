/*
 * accel.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include <stdint.h>

#include "bool.h"
#include "flight_dynamics.h"
#include "flight_math.h"
#include "sensor_align.h"
#include "filter.h"
#include "dev.h"
#include "gyro.h"

typedef struct accelerationRuntime_s {
    uint16_t accLpfCutHz;
    biquadFilter_t accFilter[XYZ_AXIS_COUNT];
    flightDynamicsTrims_t *accelerationTrims;
    int accumulatedMeasurementCount;
    float accumulatedMeasurements[XYZ_AXIS_COUNT];
    uint16_t calibratingA;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
} accelerationRuntime_t;

struct accDev_s;

typedef void (*sensorAccInitFuncPtr)(struct accDev_s *acc);
typedef bool (*sensorAccReadFuncPtr)(struct accDev_s *acc);

typedef struct accDev_s {
    float acc_1G_rec;
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    uint16_t acc_1G;
    float accADC[XYZ_AXIS_COUNT];
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e accAlign;
    bool dataReady;
    gyroDev_t *gyro;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
    uint8_t filler[2];
    fp_rotationMatrix_t rotationMatrix;
} accDev_t;

typedef struct acc_s {
    accDev_t dev;
    uint16_t sampleRateHz;
    float accADC[XYZ_AXIS_COUNT];
    bool isAccelUpdatedAtLeastOnce;
} acc_t;

typedef struct rollAndPitchTrims_s {
    int16_t roll;
    int16_t pitch;
} rollAndPitchTrims_t_def;

typedef union rollAndPitchTrims_u {
    int16_t raw[2];
    rollAndPitchTrims_t_def values;
} rollAndPitchTrims_t;

typedef struct accelerometerConfig_s {
    uint16_t acc_lpf_hz;                    // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    bool acc_high_fsr;
    flightDynamicsTrims_t accZero;
    rollAndPitchTrims_t accelerometerTrims;
} accelerometerConfig_t;

extern acc_t acc;
extern accelerationRuntime_t accelerationRuntime;
extern accelerometerConfig_t accelerometerConfig;

void accInit(sensorAccInitFuncPtr, sensorAccReadFuncPtr);
void accUpdate(void);
bool accGetAccumulationAverage(float *accumulationAverage);

#endif /* INC_ACCEL_H_ */
