/*
 * dev.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_DEV_H_
#define SRC_DEV_H_

typedef struct extiCallbackRec_s extiCallbackRec_t;
typedef void (*extiHandlerCallback)(extiCallbackRec_t *self);

struct extiCallbackRec_s {
    extiHandlerCallback *fn;
};

typedef struct extiCallbackRec_s extiCallbackRec_t;

typedef enum {
    MPU_NONE,
    MPU_3050,
    MPU_60x0,
    MPU_60x0_SPI,
    MPU_65xx_I2C,
    MPU_65xx_SPI,
    MPU_9250_SPI,
    ICM_20601_SPI,
    ICM_20602_SPI,
    ICM_20608_SPI,
    ICM_20649_SPI,
    ICM_20689_SPI,
    ICM_42605_SPI,
    ICM_42688P_SPI,
    BMI_160_SPI,
    BMI_270_SPI,
    LSM6DSO_SPI,
    L3GD20_SPI,
} mpuSensor_e;

typedef enum {
    MPU_HALF_RESOLUTION,
    MPU_FULL_RESOLUTION
} mpu6050Resolution_e;

typedef struct mpuDetectionResult_s {
    mpuSensor_e sensor;
    mpu6050Resolution_e resolution;
} mpuDetectionResult_t;

enum {
    AUTO_PROFILE_CELL_COUNT_STAY = 0, // Stay on this profile irrespective of the detected cell count. Use this profile if no other profile matches (default, i.e. auto profile switching is off)
    AUTO_PROFILE_CELL_COUNT_CHANGE = -1, // Always switch to a profile with matching cell count if there is one
};



#endif /* SRC_DEV_H_ */
