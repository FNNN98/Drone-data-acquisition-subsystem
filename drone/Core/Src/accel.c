/*
 * accel.c
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#include <string.h>

#include "accel.h"
#include "gyro.h"

acc_t acc;
accelerationRuntime_t accelerationRuntime;
accelerometerConfig_t accelerometerConfig;

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}


static void accInitFilters(void)
{
	accelerometerConfig.acc_lpf_hz = 50;

    // Only set the lowpass cutoff if the ACC sample rate is detected otherwise
    // the filter initialization is not defined (sample rate = 0)
    accelerationRuntime.accLpfCutHz = (acc.sampleRateHz) ? accelerometerConfig.acc_lpf_hz : 0;
    if (accelerationRuntime.accLpfCutHz) {
        const uint32_t accSampleTimeUs = 1e6 / acc.sampleRateHz;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInitLPF(&accelerationRuntime.accFilter[axis], accelerationRuntime.accLpfCutHz, accSampleTimeUs);
        }
    }

    accelerationRuntime.accumulatedMeasurementCount = 0;
    accelerationRuntime.accumulatedMeasurements[0] = 0.0f;
    accelerationRuntime.accumulatedMeasurements[1] = 0.0f;
    accelerationRuntime.accumulatedMeasurements[2] = 0.0f;
}

static void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationRuntime.accelerationTrims = accelerationTrimsToUse;
}

void accInit(sensorAccInitFuncPtr initFn, sensorAccReadFuncPtr readFn) {
	memset(&acc, 0, sizeof(acc));
	// copy over the common gyro mpu settings
	acc.dev.gyro = &gyro.gyroSensor1.gyroDev;
	acc.dev.acc_high_fsr = accelerometerConfig.acc_high_fsr;

	// Copy alignment from active gyro, as all production boards use acc-gyro-combi chip.
	// Exceptions are STM32F3DISCOVERY and STM32F411DISCOVERY, and (may be) handled in future enhancement.

	sensor_align_e alignment = gyro.gyroSensor1.gyroDev.gyroAlign;
	sensorAlignment_t customAlignment;
	buildAlignmentFromStandardAlignment(&customAlignment, alignment);

	acc.dev.accAlign = alignment;
	buildRotationMatrixFromAlignment(&customAlignment, &acc.dev.rotationMatrix);

	acc.dev.initFn = initFn;
	acc.dev.readFn = readFn;

	acc.dev.acc_1G = 256; // set default
	acc.dev.initFn(&acc.dev); // driver initialisation
	acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;

	acc.sampleRateHz = 1000;

	setAccelerationTrims(&accelerometerConfig.accZero);

	accInitFilters();
}

void accUpdate(void)
{
    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc.accADC[axis] = acc.dev.accADC[axis];
    }

    if (accelerationRuntime.accLpfCutHz) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            acc.accADC[axis] = biquadFilterApply(&accelerationRuntime.accFilter[axis], acc.accADC[axis]);
        }
    }

    alignSensorViaRotation(acc.accADC, acc.dev.accAlign);

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

    ++accelerationRuntime.accumulatedMeasurementCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accelerationRuntime.accumulatedMeasurements[axis] += acc.accADC[axis];
    }
}

bool accGetAccumulationAverage(float *accumulationAverage)
{
    if (accelerationRuntime.accumulatedMeasurementCount > 0) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accelerationRuntime.accumulatedMeasurements[axis] / accelerationRuntime.accumulatedMeasurementCount;
            accelerationRuntime.accumulatedMeasurements[axis] = 0.0f;
        }
        accelerationRuntime.accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}
