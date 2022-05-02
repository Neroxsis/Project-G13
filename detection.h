/*
 * detection.h
 *
 *  Created on: 27.04.2022
 *      Author: Studium
 */

#ifndef DETECTION_H
#define DETECTION_H

#include "ch.h"
#include "hal.h"

void obj_det_init(void);
uint8_t get_object_det(void);
void reset_obj_det(void);

#endif /* DETECTION_H */
