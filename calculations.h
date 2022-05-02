/*
 * calculations.h
 *
 *  Created on: 26.04.2022
 *      Author: Studium
 */

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include "sensors/imu.h"

void lookup_init(void);
float get_cos(int16_t angle);
float get_sin(int16_t angle);
int16_t get_gyro_deg(imu_msg_t *imu_values, uint8_t axis);
int8_t sign(int32_t nb);
int8_t signs(int8_t nb); // for sign short
int8_t signf(float nb);

#endif /* CALCULATIONS_H */
