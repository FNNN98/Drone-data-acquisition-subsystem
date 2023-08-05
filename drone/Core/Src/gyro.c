/*
 * gyro.c
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#include "gyro.h"

#include "debug.h"
#include "filter.h"
#include "timing.h"

gyro_t gyro;
uint8_t activePidLoopDenom = 1;

static float accumulatedMeasurements[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
static float gyroPrevious[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
static int accumulatedMeasurementCount = 0;

static void gyroSetTargetLooptime(uint8_t pidDenom)
{
    activePidLoopDenom = pidDenom;
    if (gyro.sampleRateHz) {
        gyro.sampleLooptime = 1e6 / gyro.sampleRateHz;
        gyro.targetLooptime = activePidLoopDenom * 1e6 / gyro.sampleRateHz;
    } else {
        gyro.sampleLooptime = 0;
        gyro.targetLooptime = 0;
    }
}

static uint16_t gyroSetSampleRate(gyroDev_t *gyro) {
	gyro->gyroRateKHz = GYRO_RATE_6664_Hz;
	uint16_t gyroSampleRateHz = 6664;
	uint16_t accSampleRateHz = 6664;

	gyro->mpuDividerDrops  = 0; // we no longer use the gyro's sample divider
	gyro->accSampleRateHz = accSampleRateHz;
	return gyroSampleRateHz;
}

static void gyroInitSlewLimiter(gyroSensor_t *gyroSensor) {
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;
    }
}

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor)
{
    gyroInitSlewLimiter(gyroSensor);
}

static void gyroInitSensor(gyroSensor_t *gyroSensor)
{
    gyroSensor->gyroDev.gyro_high_fsr = true;
    gyroSensor->gyroDev.gyroAlign = CW90_DEG;
    sensorAlignment_t customAlignment;
    buildAlignmentFromStandardAlignment(&customAlignment, gyroSensor->gyroDev.gyroAlign);
    buildRotationMatrixFromAlignment(&customAlignment, &gyroSensor->gyroDev.rotationMatrix);
    gyroSensor->gyroDev.mpuIntExtiTag = 0U;
    gyroSensor->gyroDev.hardware_lpf = 0U;

    // The targetLooptime gets set later based on the active sensor's gyroSampleRateHz and pid_process_denom
    gyroSensor->gyroDev.gyroSampleRateHz = gyroSetSampleRate(&gyroSensor->gyroDev);
    gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);

    // As new gyros are supported, be sure to add them below based on whether they are subject to the overflow/inversion bug
    // Any gyro not explicitly defined will default to not having built-in overflow protection as a safe alternative.
    gyroSensor->gyroDev.gyroHasOverflowProtection = false;  // default catch for newly added gyros until proven to be unaffected

    gyroInitSensorFilters(gyroSensor);
}

enum {
    FILTER_LPF1 = 0,
    FILTER_LPF2
};

static uint16_t calculateNyquistAdjustedNotchHz(uint16_t notchHz, uint16_t notchCutoffHz)
{
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / gyro.targetLooptime;
    if (notchHz > gyroFrequencyNyquist) {
        if (notchCutoffHz < gyroFrequencyNyquist) {
            notchHz = gyroFrequencyNyquist;
        } else {
            notchHz = 0;
        }
    }

    return notchHz;
}

static bool gyroInitLowpassFilterLpf(int slot, int type, uint16_t lpfHz, uint32_t looptime)
{
    filterApplyFnPtr *lowpassFilterApplyFn;
    gyroLowpassFilter_t *lowpassFilter = NULL;

    switch (slot) {
    case FILTER_LPF1:
        lowpassFilterApplyFn = &gyro.lowpassFilterApplyFn;
        lowpassFilter = gyro.lowpassFilter;
        break;

    case FILTER_LPF2:
        lowpassFilterApplyFn = &gyro.lowpass2FilterApplyFn;
        lowpassFilter = gyro.lowpass2Filter;
        break;

    default:
        return false;
    }

    bool ret = false;

    // Establish some common constants
    const uint32_t gyroFrequencyNyquist = 1000000 / 2 / looptime;
    const float gyroDt = looptime * 1e-6f;

    // Gain could be calculated a little later as it is specific to the pt1/bqrcf2/fkf branches
    const float gain = pt1FilterGain(lpfHz, gyroDt);

    // Dereference the pointer to null before checking valid cutoff and filter
    // type. It will be overridden for positive cases.
    *lowpassFilterApplyFn = nullFilterApply;

    // If lowpass cutoff has been specified
    if (lpfHz) {
        switch (type) {
        case FILTER_PT1:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterInit(&lowpassFilter[axis].pt1FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_BIQUAD:
            if (lpfHz <= gyroFrequencyNyquist) {
                *lowpassFilterApplyFn = (filterApplyFnPtr) biquadFilterApply;
                for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                    biquadFilterInitLPF(&lowpassFilter[axis].biquadFilterState, lpfHz, looptime);
                }
                ret = true;
            }
            break;
        case FILTER_PT2:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt2FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterInit(&lowpassFilter[axis].pt2FilterState, gain);
            }
            ret = true;
            break;
        case FILTER_PT3:
            *lowpassFilterApplyFn = (filterApplyFnPtr) pt3FilterApply;
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterInit(&lowpassFilter[axis].pt3FilterState, gain);
            }
            ret = true;
            break;
        }
    }
    return ret;
}

static void gyroInitFilterNotch1(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter1ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter1ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyro.notchFilter1[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void gyroInitFilterNotch2(uint16_t notchHz, uint16_t notchCutoffHz)
{
    gyro.notchFilter2ApplyFn = nullFilterApply;

    notchHz = calculateNyquistAdjustedNotchHz(notchHz, notchCutoffHz);

    if (notchHz != 0 && notchCutoffHz != 0) {
        gyro.notchFilter2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(notchHz, notchCutoffHz);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterInit(&gyro.notchFilter2[axis], notchHz, gyro.targetLooptime, notchQ, FILTER_NOTCH, 1.0f);
        }
    }
}

static void gyroInitFilters(void)
{
    gyroInitLowpassFilterLpf(
      FILTER_LPF1,
	  FILTER_PT1,
	  LPF_MAX_HZ,
      gyro.targetLooptime
    );

    gyro.downsampleFilterEnabled = gyroInitLowpassFilterLpf(
      FILTER_LPF2,
	  FILTER_PT1,
	  LPF_MAX_HZ,
      gyro.sampleLooptime
    );

    gyroInitFilterNotch1(0, 0);
    gyroInitFilterNotch2(0, 0);
}

#define PID_PROCESS_DENOM_DEFAULT       1

void gyroInit(sensorGyroInitFuncPtr initFn, sensorGyroReadFuncPtr readFn)
{
    gyro.gyroDebugMode = DEBUG_NONE;
    gyro.useDualGyroDebugging = false;
    gyro.gyroHasOverflowProtection = false;
    gyro.gyroToUse = GYRO_CONFIG_USE_GYRO_1;
    gyro.gyroDebugAxis = FD_ROLL;

	gyro.gyroSensor1.gyroDev.initFn = initFn;
	gyro.gyroSensor1.gyroDev.readFn = readFn;
    static uint8_t gyroBuf1[GYRO_BUF_SIZE];
    // SPI DMA buffer required per device
    gyro.gyroSensor1.gyroDev.dev.txBuf = gyroBuf1;
    gyro.gyroSensor1.gyroDev.dev.rxBuf = &gyroBuf1[GYRO_BUF_SIZE / 2];
    gyroInitSensor(&gyro.gyroSensor1);

    // Copy the sensor's scale to the high-level gyro object. If running in "BOTH" mode
    // then logic above requires both sensors to be the same so we'll use sensor1's scale.
    // This will need to be revised if we ever allow different sensor types to be used simultaneously.
    // Likewise determine the appropriate raw data for use in DEBUG_GYRO_RAW
    gyro.scale = gyro.gyroSensor1.gyroDev.scale;
    gyro.rawSensorDev = &gyro.gyroSensor1.gyroDev;

    if (gyro.rawSensorDev) {
        gyro.sampleRateHz = gyro.rawSensorDev->gyroSampleRateHz;
        gyro.accSampleRateHz = gyro.rawSensorDev->accSampleRateHz;
    } else {
        gyro.sampleRateHz = 0;
        gyro.accSampleRateHz = 0;
    }

	gyroSetTargetLooptime(PID_PROCESS_DENOM_DEFAULT);
	gyroInitFilters();
}

static void gyroUpdateSensor(gyroSensor_t *gyroSensor)
{
    if (!gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev)) {
        return;
    }
    gyroSensor->gyroDev.dataReady = false;

    // move 16-bit gyro data into 32-bit variables to avoid overflows in calculations
    gyroSensor->gyroDev.gyroADC[X] = gyroSensor->gyroDev.gyroADC[X] - gyroSensor->gyroDev.gyroZero[X];
    gyroSensor->gyroDev.gyroADC[Y] = gyroSensor->gyroDev.gyroADC[Y] - gyroSensor->gyroDev.gyroZero[Y];
    gyroSensor->gyroDev.gyroADC[Z] = gyroSensor->gyroDev.gyroADC[Z] - gyroSensor->gyroDev.gyroZero[Z];
    alignSensorViaRotation(gyroSensor->gyroDev.gyroADC, gyroSensor->gyroDev.gyroAlign);
}


static void gyroFilter(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // downsample the individual gyro samples
        float gyroADCf = 0;
        if (gyro.downsampleFilterEnabled) {
            // using gyro lowpass 2 filter for downsampling
            gyroADCf = gyro.sampleSum[axis];
        } else {
            // using simple average for downsampling
            if (gyro.sampleCount) {
                gyroADCf = gyro.sampleSum[axis] / gyro.sampleCount;
            }
            gyro.sampleSum[axis] = 0;
        }

        // apply static notch filters and software lowpass filters
        gyroADCf = gyro.notchFilter1ApplyFn((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
        gyroADCf = gyro.notchFilter2ApplyFn((filter_t *)&gyro.notchFilter2[axis], gyroADCf);
        gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);


        gyro.gyroADCf[axis] = gyroADCf;
    }
    gyro.sampleCount = 0;
}

static void gyroAccumulateMeasurements(void) {
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // integrate using trapezium rule to avoid bias
        accumulatedMeasurements[axis] += 0.5f * (gyroPrevious[axis] + gyro.gyroADCf[axis]) * gyro.targetLooptime;
        gyroPrevious[axis] = gyro.gyroADCf[axis];
    }
    accumulatedMeasurementCount++;
}

void gyroUpdate(void) {
	gyroUpdateSensor(&gyro.gyroSensor1);

	gyro.gyroADC[X] = gyro.gyroSensor1.gyroDev.gyroADC[X] * gyro.gyroSensor1.gyroDev.scale;
	gyro.gyroADC[Y] = gyro.gyroSensor1.gyroDev.gyroADC[Y] * gyro.gyroSensor1.gyroDev.scale;
	gyro.gyroADC[Z] = gyro.gyroSensor1.gyroDev.gyroADC[Z] * gyro.gyroSensor1.gyroDev.scale;

    if (gyro.downsampleFilterEnabled) {
        // using gyro lowpass 2 filter for downsampling
        gyro.sampleSum[X] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[X], gyro.gyroADC[X]);
        gyro.sampleSum[Y] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Y], gyro.gyroADC[Y]);
        gyro.sampleSum[Z] = gyro.lowpass2FilterApplyFn((filter_t *)&gyro.lowpass2Filter[Z], gyro.gyroADC[Z]);
    } else {
        // using simple averaging for downsampling
        gyro.sampleSum[X] += gyro.gyroADC[X];
        gyro.sampleSum[Y] += gyro.gyroADC[Y];
        gyro.sampleSum[Z] += gyro.gyroADC[Z];
        gyro.sampleCount++;
    }
    gyroFilter();
	gyroAccumulateMeasurements();
}

bool gyroGetAccumulationAverage(float *accumulationAverage)
{
    if (accumulatedMeasurementCount) {
        // If we have gyro data accumulated, calculate average rate that will yield the same rotation
        const timeUs_t accumulatedMeasurementTimeUs = accumulatedMeasurementCount * gyro.targetLooptime;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = accumulatedMeasurements[axis] / accumulatedMeasurementTimeUs;
            accumulatedMeasurements[axis] = 0.0f;
        }
        accumulatedMeasurementCount = 0;
        return true;
    } else {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            accumulationAverage[axis] = 0.0f;
        }
        return false;
    }
}

bool gyroOverflowDetected(void)
{
    return false;
}



