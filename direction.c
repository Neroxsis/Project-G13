/*
 * direction.c
 *
 *  Created on: 07.05.2022
 *      Author: Studium
 */

#include <direction.h>
#include <math.h>
#include <constants.h>
#include <sensors/imu.h>
#include <calculations.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <leds.h>

#include <chprintf.h>

#include <main.h>


extern messagebus_t bus;

static imu_msg_t imu_values;
static uint8_t picked_up = 0;
static float relative_rotation = 0;
static float distance = 0;



static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
//    imu_msg_t imu_values;
//
//    float x_speed = 0;  // mm/s
//    float y_speed = 0;  // mm/s
//
//    float x_position = 0;
//    float y_position = 0;
//
//    float period = 0.1; // because float * float faster than float * int
//
//#define ACCELERATION_NOISE_TH 42
//#define NINETY_DEGREE 90
//#define WAIT_TIME 1234
//#define ACTIVATION_TH 45245
//#define Z_ACC_THRESHOLD 0.08 //not yet calbibrated
//#define GRAVITY 9.81
//
//
//
//    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
//
//    while(1){
//		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
//
//
//		if (fabs(imu_values.acceleration[Z_AXIS]+GRAVITY) <= Z_ACC_THRESHOLD){
//			picked_up = 0;
//		} else {
//			picked_up = 1;
//		}
//
//		// calculate angle
//		relative_rotation +=  get_gyro_deg(&imu_values, Z_AXIS) * period;
//
//		// calculate speed
//		x_speed += period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
//		y_speed += period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));
//
//		// calculate distance
//		x_position += period * x_speed;
//		y_position += period * y_speed;
//
//		distance = sqrt(x_position*x_position + y_position*y_position);
//
//
//		chThdSleepMilliseconds(10);
//    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

uint8_t in_air(void){
	return picked_up;
}

float get_relative_rotation(void){
	return relative_rotation;
}

float get_distance(void){
	return distance;
}

float return_angle(float x, float y, float phi){
	float theta = 0;
	if(x){
		theta = atan2(y,x)*PI_DEG/M_PI;
	}else{
		theta = PI_DEG/2;
	}
	return (PI_DEG/2) * signf(x) + theta - phi;
}





