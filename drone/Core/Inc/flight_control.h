/*
 * flight_control.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_FLIGHT_CONTROL_H_
#define SRC_FLIGHT_CONTROL_H_

typedef enum {
    LAUNCH_CONTROL_MODE_NORMAL = 0,
    LAUNCH_CONTROL_MODE_PITCHONLY,
    LAUNCH_CONTROL_MODE_FULL,
    LAUNCH_CONTROL_MODE_COUNT // must be the last element
} launchControlMode_e;



#endif /* SRC_FLIGHT_CONTROL_H_ */
