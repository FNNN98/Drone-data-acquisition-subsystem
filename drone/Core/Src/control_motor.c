/*
 * control_motor.c
 *
 *  Created on: Jun 8, 2022
 *      Author: pietro
 */

#include "control_motor.h"

#include "pid.h"

void controlMotorInit(void) {
	pidInit();
}

void controlMotorUpdate(void) {
    pidIncUpdateCounter();
    processRcCommand();
    pidController();
}
