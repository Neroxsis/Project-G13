/*
 * detection.h
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#ifndef DETECTION_H
#define DETECTION_H

#include "ch.h"
#include "hal.h"

void obj_det_init(void);
void reset_obj_det(void);
void false_alarm(int32_t diff);
void start_detection(void);
void end_detection(void);
uint8_t get_object_det(void);

#endif /* DETECTION_H */
