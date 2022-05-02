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
void drive_steps(int32_t steps, int8_t direction);
void motor_stop(void);
void motors_reset_pos(void);

#endif /* MOTOR_C */
