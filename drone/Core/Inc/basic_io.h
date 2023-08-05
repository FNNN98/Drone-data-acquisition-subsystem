/*
 * basic_io.h
 *
 *  Created on: Jun 9, 2022
 *      Author: pietro
 */

#ifndef SRC_BASIC_IO_H_
#define SRC_BASIC_IO_H_

typedef void* IO_t;
typedef uint8_t ioTag_t;

#define IO_TAG_NONE 0
#define IO_NONE ((IO_t)0)

typedef uint8_t ioConfig_t;  // packed IO configuration

#define IO_TAG(pinid) DEFIO_TAG(pinid)
#define DEFIO_TAG(pinid) DEFIO_TAG__ ## pinid
#define DEFIO_GPIOID__A 0
#define DEFIO_TAG__NONE 0
#define DEFIO_TAG__PA0 DEFIO_TAG_MAKE(DEFIO_GPIOID__A, 0)
#define DEFIO_TAG_MAKE(gpioid, pin) ((ioTag_t)((((gpioid) + 1) << 4) | (pin)))
#define DEFIO_TAG_ISEMPTY(tag) (!(tag))
#define DEFIO_TAG_GPIOID(tag) (((tag) >> 4) - 1)
#define DEFIO_TAG_PIN(tag) ((tag) & 0x0f)

#endif /* SRC_BASIC_IO_H_ */
