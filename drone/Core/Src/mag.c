/*
 * mag.c
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#include "mag.h"
#include "bool.h"
#include "flight_dynamics.h"
#include "timing.h"
#include "sensor_align.h"

#define MAG_HMC5883_ALIGN CW270_DEG_FLIP

mag_t mag;
magDev_t magDev;

static bool doneInit = false;

typedef struct compassConfig_s {
    uint8_t mag_alignment;                  // mag alignment
    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t mag_busType;
    uint8_t mag_i2c_device;
    uint8_t mag_i2c_address;
    uint8_t mag_spi_device;
    ioTag_t mag_spi_csn;
    ioTag_t interruptTag;
    flightDynamicsTrims_t magZero;
    sensorAlignment_t mag_customAlignment;
} compassConfig_t;

static compassConfig_t compassConfig;

static bool compassDetect(magDev_t *magDev, uint8_t *alignment)
{
    *alignment = ALIGN_DEFAULT;  // may be overridden if target specifies MAG_*_ALIGN

    magSensor_e magHardware = MAG_NONE;

    extDevice_t *dev = &magDev->dev;
    // Associate magnetometer bus with its device
    dev->bus = &magDev->bus;


    switch (compassConfig.mag_busType) {
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, compassConfig.mag_i2c_device);
        dev->busType_u.i2c.address = compassConfig.mag_i2c_address;
        break;

    case BUS_TYPE_SPI:
    default:
        return false;
    }

    switch (compassConfig.mag_hardware) {
    case MAG_DEFAULT:
    case MAG_HMC5883:
        if (dev->bus->busType == BUS_TYPE_I2C) {
            dev->busType_u.i2c.address = compassConfig.mag_i2c_address;
        }

        *alignment = MAG_HMC5883_ALIGN;
        magHardware = MAG_HMC5883;
        break;

    default:
        magHardware = MAG_NONE;
        break;
    }

    // MAG_MPU925X_AK8963 is an MPU925x configured as I2C passthrough to the built-in AK8963 magnetometer
    // Passthrough mode disables the gyro/acc part of the MPU, so we only want to detect this sensor if mag_hardware was explicitly set to MAG_MPU925X_AK8963
    if (magHardware == MAG_NONE) {
        return false;
    }

    return true;
}


void magInit(sensorMagInitFuncPtr initFn, sensorMagReadFuncPtr readFn) {
	magDev.init = initFn;
	magDev.read = readFn;

	//TODO

    sensor_align_e alignment;

    if (!compassDetect(&magDev, &alignment)) {
        return;
    }

    magDev.init(&magDev);
    doneInit = true;

    magDev.magAlignment = alignment;

    if (compassConfig.mag_alignment != ALIGN_DEFAULT) {
        magDev.magAlignment = compassConfig.mag_alignment;
    }

    buildRotationMatrixFromAlignment(&compassConfig.mag_customAlignment, &magDev.rotationMatrix);
}

void magUpdate(void)
{
    if (!magDev.read(&magDev)) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    	mag.magADC[axis] = magDev.magADC[axis];
    }

    alignSensorViaRotation(mag.magADC, magDev.magAlignment);

    flightDynamicsTrims_t *magZero = &compassConfig.magZero;
    if (doneInit) {              // we apply offset only once mag calibration is done
        mag.magADC[X] -= magZero->raw[X];
        mag.magADC[Y] -= magZero->raw[Y];
        mag.magADC[Z] -= magZero->raw[Z];
    }
}
