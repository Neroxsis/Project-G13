/*
 * motor.h
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#ifndef MOTOR_C
#define MOTOR_C

#include <hal.h>
#include "ch.h"

enum dir {FORWARDS, BACKWARDS, TURN_LEFT, TURN_RIGHT};
void turn_angle(int16_t angle, uint8_t speed);
void drive_distance(float distance);
void drive_steps(int32_t steps, enum dir direction);
void motors_stop(void);
void motors_reset_pos(void);
void motors_drive_dir(enum dir direction, uint8_t fraction);
void set_found_goal(void);

#endif /* MOTOR_C */
