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
static uint8_t picked_up = 0;
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


static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    imu_msg_t imu_values;


//    float x_speed = 0;  // mm/s
//    float y_speed = 0;  // mm/s
//
//    float x_position = 0;
//    float y_position = 0;



    float period = 0.012; // because float * float faster than float * int
    float period_rot = 0.012;




#define ACCELERATION_NOISE_TH 42
#define NINETY_DEGREE 90
#define WAIT_TIME 1234
#define ACTIVATION_TH 45245
#define Z_ACC_THRESHOLD 1 //not yet calbibrated maybe even lower


#define X_ACC_THRESHOLD 0.07  // NOT yet calibrated !!
#define Y_ACC_THRESHOLD 0.07  // NOT yet calibrated !!

#define ROTATION_THRESHOLD 0.001f  // NOT yet calibrated !!
#define X_ROTATION_THRESHOLD 1.5  // NOT yet calibrated !!
#define Y_ROTATION_THRESHOLD 1.5  // NOT yet calibrated !!
#define SPEED_CORRECTION 100 // NOT yet calibrated !!

#define THRESHOLD_RETURN_ANGLE 0.2


    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");




    while(1){

    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		x_axis_acc = imu_values.acceleration[X_AXIS];
		y_axis_acc = imu_values.acceleration[Y_AXIS];
		z_axis_acc = imu_values.acceleration[Z_AXIS];


		// highest acc: led
//		clear_leds();
//
//		if (imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD){
//			set_led(LED7,1);
//		} else if (imu_values.acceleration[X_AXIS] <= - X_ACC_THRESHOLD) {
//			set_led(LED3, 1);
//		}
//
//		if (imu_values.acceleration[Y_AXIS] >= Y_ACC_THRESHOLD){
//			set_led(LED5, 1);
//		} else if (imu_values.acceleration[X_AXIS] <= - Y_ACC_THRESHOLD) {
//			set_led(LED1, 1);
//		}







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


		// y position ist forwärts genauer als rückwärts


		// calculate distance
		x_position += period * x_speed * SPEED_CORRECTION;
		y_position += period * y_speed * SPEED_CORRECTION;

		distance = sqrt(x_position*x_position + y_position*y_position);  // aus whileschleife herausnehmen !!!


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




// ------------------------------------------
//float return_angle(float x_acc, float y_acc, float phi){
//	float theta = 0;
//	float angle = 0;
//	float fraction = 0;
//	if(x_acc_displacement >= THRESHOLD_RETURN_ANGLE){
//		if(fabs(y_acc_displacement) <= Y_ACC_THRESHOLD){
//			y_acc_displacement = 0;
//		}
//		fraction = y_acc_displacement,x_acc_displacement;
//		theta = atan(fraction)*PI_DEG/M_PI;	//used to be atan2
//		//angle = (PI_DEG/2) * signf(-x_acc) + theta - phi;
//		angle = (PI_DEG/2) * (-x_acc_sign_displacement) + theta - phi;
//	}else if (x_acc_displacement <= - THRESHOLD_RETURN_ANGLE){
//		//theta = PI_DEG/2;
//		fraction = y_acc_displacement,x_acc_displacement;65+
//		theta = atan(fraction)*PI_DEG/M_PI; //used to be atan2
//
////		if(theta >= PI_DEG/2 && theta <= PI_DEG){
////			theta = -theta + PI_DEG/2;
////		}
//
//		//angle = (PI_DEG/2) * signf(-x_acc) + theta - phi;
//		angle = (PI_DEG/2) * (-x_acc_sign_displacement) + theta - phi;		//maybe use atan() gives a value between -90 and 90
//	}else if (fabs(x_acc_displacement) <= THRESHOLD_RETURN_ANGLE){
//		if(x_acc_displacement <= 0.5){
//			x_acc_displacement = 0;
//		}
//		fraction = y_acc_displacement,x_acc_displacement;
//		theta = atan(fraction)*PI_DEG/M_PI;	//used to be atan2
//		angle = (PI_DEG/2) * y_acc_sign_displacement + theta - phi;  atan2()
//	}
//	print_theta = theta;
//	return angle; // I put a minus in signf
//}


