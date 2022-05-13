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






    while(1){
    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		x_axis_acc = imu_values.acceleration[X_AXIS];
		y_axis_acc = imu_values.acceleration[Y_AXIS];
		z_axis_acc = imu_values.acceleration[Z_AXIS];


		// calculate angle

		if (fabs(get_gyro_deg(&imu_values, X_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_x += get_gyro_deg(&imu_values, X_AXIS) * period_rot;
		}

		if (fabs(get_gyro_deg(&imu_values, Y_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_y += get_gyro_deg(&imu_values, Y_AXIS) * period_rot;
		}

		if (fabs(get_gyro_deg(&imu_values, Z_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_z += get_gyro_deg(&imu_values, Z_AXIS) * period_rot;
		}


		float gravity_x = GRAVITY * (sin(relative_rotation_y) * cos(relative_rotation_z) + sin(relative_rotation_x)*sin(relative_rotation_z));
		float gravity_y = GRAVITY * (-sin(relative_rotation_x) * cos(relative_rotation_z) + sin(relative_rotation_y)*sin(relative_rotation_z));



		// CHECK if the thresholds are needed
		if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD && fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
			x_speed += period * ((imu_values.acceleration[X_AXIS]-gravity_x) * get_cos((int)relative_rotation_z) - (imu_values.acceleration[Y_AXIS]-gravity_y) * get_sin((int)relative_rotation_z));
			y_speed += period * ((imu_values.acceleration[X_AXIS]-gravity_x) * get_sin((int)relative_rotation_z) + (imu_values.acceleration[Y_AXIS]-gravity_y) * get_cos((int)relative_rotation_z));
		}

		if (!picked_up){
		    x_speed=0;
		    y_speed=0;
		}

// -----------------------------------

		// ----------------------------------------------------------------------
		//	Determines if robot is picked up and then turns on body lights
		// doesn^t work anymore if u delete the set body led in the for loops

		 if (fabs(z_axis_acc + GRAVITY) < Z_ACC_THRESHOLD){
		 	counter_small_acc++;
		 	if(counter_small_acc == 5){
		 		picked_up = 0;
		 		set_body_led(picked_up);
		 	}
		 }


		 if ((z_axis_acc + GRAVITY) <= -Z_ACC_THRESHOLD){
		 	if(picked_up == 0){
		 		picked_up = 1;
		 		set_body_led(picked_up);
		 		set_picked_up(picked_up);
		 	}else{
		 			counter_deceleration++;
		 		if (counter_deceleration==8){
		 			picked_up = 0;
		 			set_body_led(picked_up);
		 		}
		 	}
		 }

		//stop calculation as soon as it touches the ground
		 // --------------------------------------------------------------------


		 if(order != pointB){
			 if (picked_up){
				order = displacement;
				} else {
					if (order == displacement){
						order = pointB;
					}
				}
		 }


		 if(order == displacement){
		   	counter_displacement ++;
		   	chprintf((BaseSequentialStream *)&SD3, "counter displacement = %d \r\n\n", counter_displacement);
		   	if(counter_displacement == 30){
		    	chprintf((BaseSequentialStream *)&SD3, "x acc = %.2f \r\n\n", x_axis_acc);
		    	x_acc_sign_displacement = signf(x_axis_acc);
		    	x_acc_displacement = x_axis_acc;
		    	y_acc_displacement = y_axis_acc;
		    	chprintf((BaseSequentialStream *)&SD3, "y acc = %.2f \r\n\n", y_axis_acc);
		    	y_acc_sign_displacement = signf(y_axis_acc);

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
	float angle = 0;
	float fraction = 0;

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



}


