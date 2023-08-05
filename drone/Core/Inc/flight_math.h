/*
 * flight_math.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_FLIGHT_MATH_H_
#define SRC_FLIGHT_MATH_H_

#include "flight_dynamics.h"

#define M_PIf       3.14159265358979323846f
#define RAD    (M_PIf / 180.0f)
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define sq(x) ((x)*(x))

#define CONVERT_PARAMETER_TO_FLOAT(param) (0.001f * param)

typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

typedef struct fp_rotationMatrix_s {
    float m[3][3];              // matrix
} fp_rotationMatrix_t;

#define DECIDEGREES_TO_RADIANS(angle) ((angle) / 10.0f * 0.0174532925f)
#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

static inline int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

static inline float constrainf(float amt, float low, float high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}

float sin_approx(float);
float cos_approx(float);
float acos_approx(float);
float atan2_approx(float y, float x);
void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT]);
void buildRotationMatrix(fp_angles_t *delta, fp_rotationMatrix_t *rotation);
float degreesToRadians(int16_t);

#endif /* SRC_FLIGHT_MATH_H_ */
