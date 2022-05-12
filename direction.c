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

#include "msgbus/messagebus.h"


extern messagebus_t bus;

static imu_msg_t imu_values;
//static uint8_t picked_up = 0;
static float relative_rotation = 0;
static float distance = 0;

static float x_speed = 0;  // mm/s
static float y_speed = 0;  // mm/s

static float x_position = 0;
static float y_position = 0;

static float z_axis_acc = 0;



static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    imu_msg_t imu_values;

    uint16_t no_mvmt_counter = 0;

//    float x_speed = 0;  // mm/s
//    float y_speed = 0;  // mm/s
//
//    float x_position = 0;
//    float y_position = 0;



    float period = 0.05; // because float * float faster than float * int
    float period_r = 0.012;




#define ACCELERATION_NOISE_TH 42
#define NINETY_DEGREE 90
#define WAIT_TIME 1234
#define ACTIVATION_TH 45245
#define Z_ACC_THRESHOLD 1 //not yet calbibrated maybe even lower


#define X_ACC_THRESHOLD 0.0//0.05  // NOT yet calibrated !!
#define Y_ACC_THRESHOLD  0.0 //0.05  // NOT yet calibrated !!

#define Z_ROTATION_THRESHOLD 0.001f  // NOT yet calibrated !!
#define SPEED_CORRECTION 100 // NOT yet calibrated !!


    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");




    while(1){

    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		z_axis_acc = imu_values.acceleration[Z_AXIS];

		// calculate angle
		if (fabs(get_gyro_deg(&imu_values, Z_AXIS)) >= Z_ROTATION_THRESHOLD){
			relative_rotation += get_gyro_deg(&imu_values, Z_AXIS) * period_r;
		}


		//if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD && fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
			x_speed += period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
			y_speed += period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));
		//}



		// MUST be solved differently
//		if (!picked_up){
//		    x_speed=0;
//		    y_speed=0;
//		    //chprintf((BaseSequentialStream *)&SD3, "set speeds to 0");
//		}


		// y position ist forwärts genauer als rückwärts


		// calculate distance
		x_position += period * x_speed * SPEED_CORRECTION;
		y_position += period * y_speed * SPEED_CORRECTION;

		distance = sqrt(x_position*x_position + y_position*y_position);  // aus whileschleife herausnehmen !!!


		chThdSleepUntilWindowed(time, time + MS2ST(12));

    }



}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

//uint8_t in_air(void){
//	return picked_up;
//}

float get_relative_rotation(void){
	return relative_rotation;
}

float get_distance(void){
	return distance;
}

float get_x_speed(void){
	return x_speed;
}

float get_y_speed(void){
	return y_speed;
}

float get_x_position(void){
	return x_position;
}

float get_z_axis_acc(void){
	return z_axis_acc;
}

float get_y_position(void){
	return y_position;
}

uint8_t incline_detected(void){
	if(imu_values.acceleration[Z_AXIS] < INCLINE_TH){
		return 1;
	}
	return 0;
}

int16_t angle_to_gradient(void){
	int16_t angle = 0;
	if(incline_detected()){
		if(fabs(imu_values.acceleration[Y_AXIS]) < DIVIDE_BYZ){
			angle = 90;
		}else{
			angle = (int) ((PI_DEG / M_PI) * atan2(imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS]));
		}
	}
	return angle;
}



