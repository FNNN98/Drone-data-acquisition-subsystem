/*
 * filter.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

#include <stdint.h>

#define LPF_MAX_HZ 1000

struct filter_s;
typedef struct filter_s filter_t;

typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef struct pt2Filter_s {
    float state;
    float state1;
    float k;
} pt2Filter_t;

typedef struct pt3Filter_s {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;

typedef struct slewFilter_s {
    float state;
    float slewLimit;
    float threshold;
} slewFilter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float weight;
} biquadFilter_t;

typedef union gyroLowpassFilter_u {
    pt1Filter_t pt1FilterState;
    biquadFilter_t biquadFilterState;
    pt2Filter_t pt2FilterState;
    pt3Filter_t pt3FilterState;
} gyroLowpassFilter_t;

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
    pt2Filter_t pt2Filter;
    pt3Filter_t pt3Filter;
} dtermLowpass_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
} lowpassFilterType_e;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

float filterGetNotchQ(float centerFreq, float cutoffFreq);
float nullFilterApply(filter_t *, float);
float pt1FilterGain(float f_cut, float dT);
void pt1FilterInit(pt1Filter_t *, float);
float pt1FilterApply(pt1Filter_t *, float);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
float biquadFilterApply(biquadFilter_t *, float);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType, float weight);
float pt2FilterGain(float f_cut, float dT);
void pt2FilterInit(pt2Filter_t *, float);
float pt2FilterApply(pt2Filter_t *, float);
float pt3FilterGain(float f_cut, float dT);
void pt3FilterInit(pt3Filter_t *, float);
float pt3FilterApply(pt3Filter_t *, float);


#endif /* SRC_FILTER_H_ */
