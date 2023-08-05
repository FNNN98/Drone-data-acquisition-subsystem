/*
 * sensor_align.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_SENSOR_ALIGN_H_
#define SRC_SENSOR_ALIGN_H_

#include <stdint.h>
#include "flight_math.h"

#define ALIGNMENT_ROTATION_WIDTH 2
#define ALIGNMENT_TO_BITMASK(alignment) ((alignment - CW0_DEG) & 0x3) | (((alignment - CW0_DEG) & 0x4) << 1)
#define ALIGNMENT_AXIS_ROTATIONS_MASK(axis) (0x3 << ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))
#define ALIGNMENT_AXIS_ROTATIONS(bits, axis) ((bits & ALIGNMENT_AXIS_ROTATIONS_MASK(axis)) >> ((FD_YAW - axis) * ALIGNMENT_ROTATION_WIDTH))

typedef enum {
    ALIGN_DEFAULT = 0, // driver-provided alignment

    // the order of these 8 values also correlate to corresponding code in ALIGNMENT_TO_BITMASK.

                            // R, P, Y
    CW0_DEG = 1,            // 00,00,00
    CW90_DEG = 2,           // 00,00,01
    CW180_DEG = 3,          // 00,00,10
    CW270_DEG = 4,          // 00,00,11
    CW0_DEG_FLIP = 5,       // 00,10,00 // _FLIP = 2x90 degree PITCH rotations
    CW90_DEG_FLIP = 6,      // 00,10,01
    CW180_DEG_FLIP = 7,     // 00,10,10
    CW270_DEG_FLIP = 8,     // 00,10,11

    ALIGN_CUSTOM = 9,    // arbitrary sensor angles, e.g. for external sensors
} sensor_align_e;

typedef union sensorAlignment_u {
    // value order is the same as axis_e

    // values are in DECIDEGREES, and should be limited to +/- 3600

    int16_t raw[XYZ_AXIS_COUNT];
    struct {
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    };
} sensorAlignment_t;

void buildRotationMatrixFromAlignment(const sensorAlignment_t*, fp_rotationMatrix_t*);
void buildAlignmentFromStandardAlignment(sensorAlignment_t*, sensor_align_e);
void alignSensorViaRotation(float *dest, uint8_t rotation);

#endif /* SRC_SENSOR_ALIGN_H_ */
