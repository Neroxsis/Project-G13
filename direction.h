/*
 * direction.h
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#ifndef DIRECTION_H_
#define DIRECTION_H_

#include "ch.h"
#include "hal.h"

static enum order {pointA, displacement, pointB};

void direction_init(void);
int16_t get_distance(void);
void set_state(enum order state_);
int8_t check_state_pointB(void);
void reset_direction(void);
int16_t get_angle(void);
void set_leds1357(int8_t);

#endif /* DIRECTION_H_ */
