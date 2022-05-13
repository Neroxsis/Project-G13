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
float get_relative_rotation_x(void);
float get_relative_rotation_y(void);
float get_relative_rotation_z(void);
float get_distance(void);
float get_x_axis_acc(void);
float get_y_axis_acc(void);
float get_z_axis_acc(void);
float get_x_speed(void);
float get_y_speed(void);
float get_x_position(void);
float get_y_position(void);
float return_angle(float x, float y, float phi);

int8_t get_x_acc_sign_displacement(void);
void set_x_acc_sign_displacement(int8_t acc_sign_displacement_t);

int8_t get_y_acc_sign_displacement(void);
void set_y_acc_sign_displacement(int8_t acc_sign_displacement_t);

float get_x_acc_displacement(void);
void set_x_acc_displacement(float x_acc_displacement_);

float get_y_acc_displacement(void);
void set_y_acc_displacement(float y_acc_displacement_);

float get_print_theta(void);


#endif /* DIRECTION_H_ */
