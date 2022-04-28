/*
 * motor.h
 *
 *  Created on: 28.04.2022
 *      Author: Studium
 */

#ifndef MOTOR_C
#define MOTOR_C

#include <hal.h>

void turn_angle(int16_t angle);
void drive_distance(float distance);
void motor_stop(void);


#endif /* MOTOR_C */
