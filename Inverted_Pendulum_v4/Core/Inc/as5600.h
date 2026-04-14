/*
 * as5600.h
 *
 *  Created on: Mar 13, 2026
 *      Author: dalya
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_
#include "main.h"
#define AS5600_ADDR  (0x36 << 1)
#define AS5600_ANGLE_REG 0x0E

float AS5600_ReadAngle(void);
uint16_t AS5600_ReadTicks(void);

#endif /* INC_AS5600_H_ */
