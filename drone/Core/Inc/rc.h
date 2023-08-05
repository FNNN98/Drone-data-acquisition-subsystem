/*
 * rc.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_RC_H_
#define SRC_RC_H_

#define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100
#define MAX_SUPPORTED_RC_CHANNEL_COUNT              18
#define RC_RX_RATE_MIN_US                       950   // 0.950ms to fit 1kHz without an issue
#define RC_RX_RATE_MAX_US                       65500 // 65.5ms or 15.26hz
#define SETPOINT_RATE_LIMIT 1998

typedef enum rc_alias {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4,
    AUX5,
    AUX6,
    AUX7,
    AUX8,
    AUX9,
    AUX10,
    AUX11,
    AUX12
} rc_alias_e;

#define MAX_RATE_PROFILE_NAME_LENGTH 8u
typedef struct controlRateConfig_s {
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates_type;
    uint8_t rcRates[3];
    uint8_t rcExpo[3];
    uint8_t rates[3];
    uint8_t tpa_rate;                       // Percent reduction in P or D at full throttle
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t throttle_limit_type;            // Sets the throttle limiting type - off, scale or clip
    uint8_t throttle_limit_percent;         // Sets the maximum pilot commanded throttle limit
    uint16_t rate_limit[3];                 // Sets the maximum rate for the axes
    uint8_t tpaMode;                        // Controls which PID terms TPA effects
    char profileName[MAX_RATE_PROFILE_NAME_LENGTH + 1]; // Descriptive name for rate profile
    uint8_t quickRatesRcExpo;               // Sets expo on rc command for quick rates
    uint8_t levelExpo[2];                   // roll/pitch level mode expo
} controlRateConfig_t;

#endif /* SRC_RC_H_ */
