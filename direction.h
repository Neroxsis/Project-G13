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

void direction_init(void);
int16_t get_distance(void);
void set_order(int8_t i);
int8_t check_order_pointB(void);
void reset_direction(void);
int16_t get_angle(void);
void set_leds1357(int8_t);



#endif /* DIRECTION_H_ */
