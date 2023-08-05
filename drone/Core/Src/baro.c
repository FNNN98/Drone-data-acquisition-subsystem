/*
 * barometer.c
 *
 *  Created on: 25 mag 2023
 *      Author: pietro
 */

#include <math.h>

#include "baro.h"
#include "basic_io.h"
#include "comm.h"

#define BARO_I2C_INSTANCE I2CINVALID
#define DEFAULT_BARO_I2C_ADDRESS 0
#define BARO_EOC_PIN NONE
#define BARO_XCLR_PIN NONE

baro_t baro;

typedef struct barometerConfig_s {
    uint8_t baro_busType;
    uint8_t baro_spi_device;
    ioTag_t baro_spi_csn;                   // Also used as XCLR (positive logic) for BMP085
    uint8_t baro_i2c_device;
    uint8_t baro_i2c_address;
    uint8_t baro_hardware;                  // Barometer hardware to use
    ioTag_t baro_eoc_tag;
    ioTag_t baro_xclr_tag;
} barometerConfig_t;

barometerConfig_t barometerConfig_System;

static inline const barometerConfig_t* barometerConfig(void) { return &barometerConfig_System; }

static void pgResetFn_barometerConfig(barometerConfig_t *barometerConfig)
{
    barometerConfig->baro_hardware = BARO_LPS;
    barometerConfig->baro_busType = BUS_TYPE_I2C;
    barometerConfig->baro_i2c_device = I2C_DEV_TO_CFG(BARO_I2C_INSTANCE);
    barometerConfig->baro_i2c_address = DEFAULT_BARO_I2C_ADDRESS;
    barometerConfig->baro_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    barometerConfig->baro_spi_csn = IO_TAG_NONE;
    barometerConfig->baro_eoc_tag = IO_TAG(BARO_EOC_PIN);
    barometerConfig->baro_xclr_tag = IO_TAG(BARO_XCLR_PIN);
}

static void lpsNothing(baroDev_t *baro)
{
    (void)(baro);
    return;
}

static bool lpsNothingBool(baroDev_t *baro)
{
	(void)(baro);
    return true;
}

static void lpsNothingCalculate(int32_t *pressure, int32_t *temperature)
{
	(void)(*pressure);
	(void)(*temperature);
    return;
}

static bool lpsDetect(baroDev_t *baro)
{
    baro->combined_read = true;
    baro->ut_delay = 1;
    baro->up_delay = 1000000 / 24;
    baro->start_ut = lpsNothing;
    baro->get_ut = lpsNothingBool;
    baro->read_ut = lpsNothingBool;
    baro->start_up = lpsNothing;
    baro->get_up = lpsNothingBool;
    baro->read_up = lpsNothingBool;
    baro->calculate = lpsNothingCalculate;
    return true;
}

static bool baroDetect(baroDev_t *baroDev, baroSensor_e baroHardwareToUse)
{
    extDevice_t *dev = &baroDev->dev;

    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function

    baroSensor_e baroHardware = baroHardwareToUse;

    switch (barometerConfig()->baro_busType) {
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, barometerConfig()->baro_i2c_device);
        dev->busType_u.i2c.address = barometerConfig()->baro_i2c_address;
        break;
    default:
        return false;
    }

    switch (baroHardware) {
    case BARO_DEFAULT:
    case BARO_BMP085:
    case BARO_MS5611:
    case BARO_LPS:
        if (lpsDetect(baroDev)) {
            baroHardware = BARO_LPS;
            break;
        }
    case BARO_DPS310:
    case BARO_BMP388:
    case BARO_BMP280:
    case BARO_QMP6988:
    case BARO_2SMPB_02B:
    case BARO_VIRTUAL:
    case BARO_NONE:
        baroHardware = BARO_NONE;
        break;
    }

    if (baroHardware == BARO_NONE) {
        return false;
    }

    return true;
}

static float pressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

void baroInit(sensorBaroInitFuncPtr initFn, sensorBaroReadFuncPtr readFn)
{
	pgResetFn_barometerConfig(&barometerConfig_System);
    baroDetect(&baro.dev, barometerConfig()->baro_hardware);
    baro.dev.read_ut = readFn;
    initFn(&baro.dev);
    if (baro.dev.baroADC > 0) {
    	baro.ground_altitude = pressureToAltitude(baro.dev.baroADC);
    } else {
    	baro.ground_altitude = 0;
    }
}

void baroUpdate(void)
{
    baro.dev.read_ut(&baro.dev);
    if (baro.dev.baroADC > 0) {
        const float altitude = pressureToAltitude(baro.dev.baroADC);
        baro.altitude = altitude - baro.ground_altitude;
    }
}
