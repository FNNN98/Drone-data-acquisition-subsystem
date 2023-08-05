/*
 * barometer.h
 *
 *  Created on: 25 mag 2023
 *      Author: pietro
 */

#ifndef INC_BARO_H_
#define INC_BARO_H_

#include "comm.h"
#include "dev.h"

typedef enum {
    BARO_DEFAULT = 0,
    BARO_NONE = 1,
    BARO_BMP085 = 2,
    BARO_MS5611 = 3,
    BARO_BMP280 = 4,
    BARO_LPS = 5,
    BARO_QMP6988 = 6,
    BARO_BMP388 = 7,
    BARO_DPS310 = 8,
    BARO_2SMPB_02B = 9,
    BARO_VIRTUAL = 10,
} baroSensor_e;

struct baroDev_s;

typedef void (*sensorBaroInitFuncPtr)(struct baroDev_s *baro);
typedef bool (*sensorBaroReadFuncPtr)(struct baroDev_s *baro);
typedef void (*sensorBaroCalculateFuncPtr)(int32_t *baroADC, int32_t *temperature);

// the 'u' in these variable names means 'uncompensated', 't' is temperature, 'p' pressure.
typedef struct baroDev_s {
    extDevice_t dev;
    extiCallbackRec_t exti;
    bool combined_read;
    uint16_t ut_delay;
    uint16_t up_delay;
    sensorBaroInitFuncPtr start_ut;
    sensorBaroReadFuncPtr read_ut;
    sensorBaroReadFuncPtr get_ut;
    sensorBaroInitFuncPtr start_up;
    sensorBaroReadFuncPtr read_up;
    sensorBaroReadFuncPtr get_up;
    sensorBaroCalculateFuncPtr calculate;
    int32_t baroADC;
    int32_t temperature; //unused
} baroDev_t;

typedef struct baro_s {
    baroDev_t dev;
    float ground_altitude;
    float altitude;
} baro_t;

extern baro_t baro;

void baroInit(sensorBaroInitFuncPtr, sensorBaroReadFuncPtr);
void baroUpdate(void);

#endif /* INC_BARO_H_ */
