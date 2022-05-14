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
void set_picked_up(uint8_t picked_up_);
float get_relative_rotation_z(void);
void set_relative_rotation_z(float rotation);
float get_distance(void);
float get_z_axis_acc(void);
float return_angle(float x, float y, float angle);

float get_x_acc_displacement(void);
void set_x_acc_displacement(float x_acc_displacement_);

float get_y_acc_displacement(void);
void set_y_acc_displacement(float y_acc_displacement_);

void set_y_acc_sign_displacement(int8_t sign);
void set_x_acc_sign_displacement(int8_t sign);

float get_print_theta(void);

uint8_t get_picked_up(void);
void set_picked_up(uint8_t picked_up_);

enum state get_order(void);
void set_order(int8_t i);

float get_save_return_angle(void);
void set_save_return_angle(float angle);

int16_t get_counter_displacement(void);
void set_counter_displacement(int16_t counter);

int8_t check_order_pointB(void);

float get_x_acc(void);
float get_y_acc(void);



#endif /* DIRECTION_H_ */
