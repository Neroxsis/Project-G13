/*
 * motor.h
 *
 *  Created on: 28.04.2022
 *      Author: Studium
 */

#ifndef MOTOR_C
#define MOTOR_C

#include <hal.h>
#include "ch.h"

enum dir {FORWARDS, BACKWARDS, TURN_LEFT, TURN_RIGHT};
void turn_angle(int16_t angle);
void drive_distance(float distance);
void drive_steps(int32_t steps, enum dir direction);
void motors_stop(void);
void motors_reset_pos(void);
void motors_drive_dir(enum dir direction, uint8_t fraction);

#endif /* MOTOR_C */
