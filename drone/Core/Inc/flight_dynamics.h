/*
 * flight_dynamics.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_FLIGHT_DYNAMICS_H_
#define SRC_FLIGHT_DYNAMICS_H_

#include <stdint.h>

#define XYZ_AXIS_COUNT 3
#define FLIGHT_DYNAMICS_INDEX_COUNT 3

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

typedef enum {
    FD_ROLL = 0,
    FD_PITCH,
    FD_YAW
} flight_dynamics_index_t;


typedef struct int16_flightDynamicsTrims_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t calibrationCompleted;
} flightDynamicsTrims_def_t;

typedef union flightDynamicsTrims_u {
    int16_t raw[4];
    flightDynamicsTrims_def_t values;
} flightDynamicsTrims_t;


#endif /* SRC_FLIGHT_DYNAMICS_H_ */
