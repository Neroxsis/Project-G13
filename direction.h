/*
 * direction.h
 *
 *  Created on: 07.05.2022
 *      Author: Studium
 */

#ifndef DIRECTION_H_
#define DIRECTION_H_

#include "ch.h"
#include "hal.h"

void direction_init(void);
uint8_t in_air(void);
uint8_t incline_detected(void);
int16_t angle_to_gradient(void);
float get_relative_rotation(void);
float get_distance(void);


#endif /* DIRECTION_H_ */
