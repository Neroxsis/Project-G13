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

static float relative_rotation_x = 0;
static float relative_rotation_y = 0;
static float relative_rotation_z = 0;
static float distance = 0;

static float x_speed = 0;  // mm/s
static float y_speed = 0;  // mm/s

static float x_position = 0;
static float y_position = 0;

static float x_axis_acc = 0;
static float y_axis_acc = 0;
static float z_axis_acc = 0;
static float x_acc_sign_displacement = 0;
static float y_acc_sign_displacement = 0;
static float x_acc_displacement = 0;
static float y_acc_displacement = 0;
static float print_theta = 0;

static uint8_t picked_up = 0;
static int16_t counter_displacement = 0;
static int16_t save_return_angle = 0;

static enum state {pointA, displacement, pointB};
static enum state order = pointA;


static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    imu_msg_t imu_values;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    float period = 0.012; // because float * float faster than float * int
    float period_rot = 0.012;
    order = pointA;
    int8_t counter_deceleration = 0;
    int8_t counter_small_acc = 0;


#define ACCELERATION_NOISE_TH 42
#define NINETY_DEGREE 90
#define WAIT_TIME 1234
#define ACTIVATION_TH 45245
#define X_ACC_THRESHOLD 0.07
#define Y_ACC_THRESHOLD 0.07
#define ROTATION_THRESHOLD 0.001f
#define X_ROTATION_THRESHOLD 1.5
#define Y_ROTATION_THRESHOLD 1.5
#define SPEED_CORRECTION 100
#define THRESHOLD_RETURN_ANGLE 0.2




    while(1){ //infinite loop

    	// calculation relative_rotation
    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		if (fabs(get_gyro_deg(&imu_values, Z_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_z += get_gyro_deg(&imu_values, Z_AXIS) * period_rot;
		}


		// ----------------------------------------------------------------------
		//	Determines if robot is picked up and then turns on body lights
		// doesn't work anymore if u delete the set body led in the for loops

		// checks if robot is on the ground
		 if (fabs(imu_values.acceleration[Z_AXIS] + GRAVITY) < Z_ACC_THRESHOLD){
		 	counter_small_acc++;
		 	if(counter_small_acc == 5){
		 		picked_up = 0;
		 		set_body_led(picked_up);
		 	}
		 }


		 // do it with x,y, acc
		 // maybe adjust THRESHOLD..
		 // checks if robot picked up or put down
//		 if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD &&
//				 fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
//		 	if(picked_up == 0){
//		 		picked_up = 1;
//		 		set_body_led(picked_up);
//		 		set_picked_up(picked_up);
//
//		 	}else{
//		 			counter_deceleration++;
//		 		if (counter_deceleration==8){
//		 			picked_up = 0;
//		 			set_body_led(picked_up);
//		 		}
//		 	}
//		 }

		 //pick up
		 if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD &&
		 				 fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
		 	picked_up = 1;
		 	set_body_led(picked_up);
		 	set_picked_up(picked_up);
		 }

		 // put down
		 if (fabs(imu_values.acceleration[X_AXIS]) <= X_ACC_THRESHOLD &&
				 fabs(imu_values.acceleration[Y_AXIS]) <= Y_ACC_THRESHOLD){
		 	picked_up = 0;
		 	set_body_led(picked_up);
		 	set_picked_up(picked_up);
		 }


		// pointB if robot is put down
		 if(order != pointB){
			 if (picked_up){
				order = displacement;
			 } else {
				if (order == displacement){
					order = pointB;
				}
			 }
		 }


		 //return angle is calculated here
		 if(order == displacement){
		   	counter_displacement ++;
		   	if(counter_displacement == 30){
		    	x_acc_sign_displacement = signf(imu_values.acceleration[X_AXIS]);
		    	x_acc_displacement = imu_values.acceleration[X_AXIS];
		    	y_acc_displacement = imu_values.acceleration[Y_AXIS];
		    	y_acc_sign_displacement = signf(imu_values.acceleration[Y_AXIS]);
		    	save_return_angle = return_angle(get_x_axis_acc(), get_y_axis_acc(), get_relative_rotation_z());
		   	}
		 }//end if

		 chThdSleepUntilWindowed(time, time + MS2ST(12));	// IMU reads new values every 250 Hz -> Source CITE !!!


    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

void set_picked_up(uint8_t picked_up_){
	picked_up = picked_up_;
}

float get_relative_rotation_x(void){
	return relative_rotation_x;
}

float get_relative_rotation_y(void){
	return relative_rotation_y;
}

float get_relative_rotation_z(void){
	return relative_rotation_z;
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

float get_y_axis_acc(void){
	return y_axis_acc;
}

float get_x_axis_acc(void){
	return x_axis_acc;
}

float get_y_position(void){
	return y_position;
}


//might not need this one
int8_t get_x_acc_sign_displacement(void){
	return x_acc_sign_displacement;
}

void set_x_acc_sign_displacement(int8_t x_acc_sign_displacement_){
	x_acc_sign_displacement = x_acc_sign_displacement_;
}

float get_x_acc_displacement(void){
	return x_acc_displacement;
}

void set_x_acc_displacement(float x_acc_displacement_){
	x_acc_displacement = x_acc_displacement_;
}

float get_y_acc_displacement(void){
	return y_acc_displacement;
}

void set_y_acc_displacement(float y_acc_displacement_){
	y_acc_displacement = y_acc_displacement_;
}

int8_t get_y_acc_sign_displacement(void){
	return y_acc_sign_displacement;
}

float get_print_theta(void){
	return print_theta;
}

void set_y_acc_sign_displacement(int8_t y_acc_sign_displacement_){
	y_acc_sign_displacement = y_acc_sign_displacement_;
}

uint8_t get_picked_up(void){
	return picked_up;
}



enum state get_order(void){
	return order;
}

void set_order(int8_t i){
	if (i==0){order=pointA;}
	if (i==1){order=displacement;}
	if (i==2){order=pointB;}
}

float get_save_return_angle(void){
	return save_return_angle;
}

void set_save_return_angle(float angle){
	save_return_angle = angle;
}

int16_t get_counter_displacement(void){
	return counter_displacement;
}

void set_counter_displacement(int16_t counter){
	counter_displacement = counter;
}

int8_t check_order_pointB(void){
	if(order == pointB){
		return 1;
	}else{
		return 0;
	}
}


// phi = relative_rotation_z
//or I could just take the acc during the displacement and then take the sign here...
//rethink the stuff I'm giving as parameters ... !!
float return_angle(float x_acc, float y_acc, float phi){
	float theta = 0;

	theta = atan2(y_acc_displacement, x_acc_displacement)*PI_DEG/M_PI;
	print_theta = theta;

	if(x_acc_sign_displacement == 1){

		if(y_acc_sign_displacement == 1){	// quadrant I  OK
			return -90 + theta;
		} else {							// quadrant IV
			return - 180 - theta;
		}
	}


	if(x_acc_sign_displacement == -1){

		if(y_acc_sign_displacement == 1){	// quadrant II  OK
			return -90 + theta;
		} else {							// quadrant III Wrong: N, gernerally maybe a bit off
			return 90 + (180 + theta); // used to be -
		}
	}

	return 0;

}


