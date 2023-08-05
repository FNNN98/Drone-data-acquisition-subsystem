/*
 * comm.c
 *
 *  Created on: Jun 10, 2022
 *      Author: pietro
 */

#include "comm.h"

#define I2CDEV_COUNT 4
#define I2C_CFG_TO_DEV(x)   ((x) - 1)

bool i2cBusSetInstance(extDevice_t *dev, uint32_t device)
{
    // I2C bus structures to associate with external devices
    static busDevice_t i2cBus[I2CDEV_COUNT];

    if ((device < 1) || (device > I2CDEV_COUNT)) {
        return false;
    }

    dev->bus = &i2cBus[I2C_CFG_TO_DEV(device)];
    dev->bus->busType = BUS_TYPE_I2C;
    dev->bus->busType_u.i2c.device = I2C_CFG_TO_DEV(device);

    return true;
}
