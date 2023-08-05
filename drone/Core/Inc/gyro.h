/*
 * gyro.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_GYRO_H_
#define SRC_GYRO_H_

#include "flight_dynamics.h"
#include "sensor_align.h"
#include "comm.h"
#include "dev.h"
#include "filter.h"

#define GYRO_CONFIG_USE_GYRO_1      0
#define GYRO_BUF_SIZE 32

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_6400_Hz,
    GYRO_RATE_6664_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef enum {
    GYRO_EXTI_INIT = 0,
    GYRO_EXTI_INT_DMA,
    GYRO_EXTI_INT,
    GYRO_EXTI_NO_INT
} gyroModeSPI_e;

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20601,
    GYRO_ICM20602,
    GYRO_ICM20608G,
    GYRO_ICM20649,
    GYRO_ICM20689,
    GYRO_ICM42605,
    GYRO_ICM42688P,
    GYRO_BMI160,
    GYRO_BMI270,
    GYRO_LSM6DSO,
    GYRO_FAKE
} gyroHardware_e;

struct gyroDev_s;

typedef void (*sensorGyroInitFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadFuncPtr)(struct gyroDev_s *gyro);
typedef bool (*sensorGyroReadDataFuncPtr)(struct gyroDev_s *gyro, int16_t *data);

typedef struct gyroDev_s {
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    extiCallbackRec_t exti;
    extDevice_t dev;
    float scale;                                             // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
    int16_t temperature;
    mpuDetectionResult_t mpuDetectionResult;
    sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    gyroModeSPI_e gyroModeSPI;
    volatile bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    uint8_t mpuDividerDrops;
    ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    gyroHardware_e gyroHardware;
    fp_rotationMatrix_t rotationMatrix;
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;
    uint8_t accDataReg;
    uint8_t gyroDataReg;
} gyroDev_t;

typedef struct gyroCalibration_s {
    float sum[XYZ_AXIS_COUNT];
    stdev_t var[XYZ_AXIS_COUNT];
    int32_t cyclesRemaining;
} gyroCalibration_t;

typedef struct gyroSensor_s {
    gyroDev_t gyroDev;
    gyroCalibration_t calibration;
} gyroSensor_t;

typedef struct gyro_s {
    uint16_t sampleRateHz;
    uint32_t targetLooptime;
    uint32_t sampleLooptime;
    float scale;
    float gyroADC[XYZ_AXIS_COUNT];     // aligned, calibrated, scaled, but unfiltered data from the sensor(s)
    float gyroADCf[XYZ_AXIS_COUNT];    // filtered gyro data
    uint8_t sampleCount;               // gyro sensor sample counter
    float sampleSum[XYZ_AXIS_COUNT];   // summed samples used for downsampling
    bool downsampleFilterEnabled;      // if true then downsample using gyro lowpass 2, otherwise use averaging

    gyroSensor_t gyroSensor1;

    gyroDev_t *rawSensorDev;           // pointer to the sensor providing the raw data for DEBUG_GYRO_RAW

    // lowpass gyro soft filter
    filterApplyFnPtr lowpassFilterApplyFn;
    gyroLowpassFilter_t lowpassFilter[XYZ_AXIS_COUNT];

    // lowpass2 gyro soft filter
    filterApplyFnPtr lowpass2FilterApplyFn;
    gyroLowpassFilter_t lowpass2Filter[XYZ_AXIS_COUNT];

    // notch filters
    filterApplyFnPtr notchFilter1ApplyFn;
    biquadFilter_t notchFilter1[XYZ_AXIS_COUNT];

    filterApplyFnPtr notchFilter2ApplyFn;
    biquadFilter_t notchFilter2[XYZ_AXIS_COUNT];

    uint16_t accSampleRateHz;
    uint8_t gyroToUse;
    uint8_t gyroDebugMode;
    bool gyroHasOverflowProtection;
    bool useDualGyroDebugging;
    flight_dynamics_index_t gyroDebugAxis;

} gyro_t;

extern gyro_t gyro;
extern uint8_t activePidLoopDenom;

void gyroInit(sensorGyroInitFuncPtr, sensorGyroReadFuncPtr);
void gyroUpdate(void);
bool gyroGetAccumulationAverage(float *accumulationAverage);
bool gyroOverflowDetected(void);

#endif /* SRC_GYRO_H_ */
