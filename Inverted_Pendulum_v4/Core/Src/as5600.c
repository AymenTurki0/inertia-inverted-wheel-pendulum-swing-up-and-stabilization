/*
 * as5600.c
 *
 *  Created on: Mar 13, 2026
 *      Author: dalya
 */

//#include "as5600.h"
//#include "pendulum.h"
//#include "main.h"
//extern I2C_HandleTypeDef hi2c1;
//
//uint16_t AS5600_ReadTicks(void)
//{
//    uint8_t buffer[2];
//    uint16_t relative_angle;
//    HAL_I2C_Mem_Read(&hi2c1,
//                     AS5600_ADDR,
//                     AS5600_ANGLE_REG,
//                     I2C_MEMADD_SIZE_8BIT,
//                     buffer,
//                     2,
//                     HAL_MAX_DELAY);
//
//    relative_angle = ((uint16_t)buffer[0] << 8) | buffer[1];
//    relative_angle = relative_angle & 0x0FFF;   // 12-bit value (0-4095)
//    return relative_angle;
//}
//
//float AS5600_ReadAngle(void)
//{
//    uint16_t relative_angle = AS5600_ReadTicks();
//    float absolute_angle = (float)(relative_angle)/M_CPR;
//    return absolute_angle;
//}
