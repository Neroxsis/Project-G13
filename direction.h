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
float get_relative_rotation(void);
float get_distance(void);
float return_angle(float x, float y, float phi);


#endif /* DIRECTION_H_ */
