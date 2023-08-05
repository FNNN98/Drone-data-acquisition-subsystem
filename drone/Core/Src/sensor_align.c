/*
 * sensor_align.c
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#include <string.h>
#include "sensor_align.h"

void buildRotationMatrixFromAlignment(const sensorAlignment_t* sensorAlignment, fp_rotationMatrix_t* rm)
{
    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = DECIDEGREES_TO_RADIANS(sensorAlignment->roll);
    rotationAngles.angles.pitch = DECIDEGREES_TO_RADIANS(sensorAlignment->pitch);
    rotationAngles.angles.yaw   = DECIDEGREES_TO_RADIANS(sensorAlignment->yaw);

    buildRotationMatrix(&rotationAngles, rm);
}

void buildAlignmentFromStandardAlignment(sensorAlignment_t* sensorAlignment, sensor_align_e alignment)
{
    if (alignment == ALIGN_CUSTOM || alignment == ALIGN_DEFAULT) {
        return;
    }

    uint8_t alignmentBits = ALIGNMENT_TO_BITMASK(alignment);

    memset(sensorAlignment, 0x00, sizeof(sensorAlignment_t));

    for (int axis = 0; axis < FLIGHT_DYNAMICS_INDEX_COUNT; axis++) {
        sensorAlignment->raw[axis] = DEGREES_TO_DECIDEGREES(90) * ALIGNMENT_AXIS_ROTATIONS(alignmentBits, axis);
    }
}

void alignSensorViaRotation(float *dest, uint8_t rotation)
{
    const float x = dest[X];
    const float y = dest[Y];
    const float z = dest[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }
}


