/*
 * motor.c
 *
 *  Created on: 28.04.2022
 *      Author: Studium
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arm_math.h>
#include <constants.h>
#include <calculations.h>
#include <motor.h>
#include <detection.h>

#include "motors.h"


#define ONE_TURN_STEPS 1000
#define WHEEL_PERIM 13 //cm
#define TURN_SPEED 700
#define DRIVE_SPEED 900

/******************************Private Functions********************************/
int32_t evade_obj_alg(void){
	static int32_t old_pos = 0;
	old_pos = right_motor_get_pos();
	do{
		switch(get_object_det()){
			case 0:
				return old_pos; // definitely needs some calculations depending on the size of the object
				break;
			case 1:
				//turn left slowly
				right_motor_set_speed(TURN_SPEED/2);
				left_motor_set_speed(-TURN_SPEED/2);
				while(get_object_det() == 1){
					chThdSleepMilliseconds(50);
				}
				break;
			case 2:
				//drive forward slowly until sensors lose object
				right_motor_set_speed(DRIVE_SPEED/2);
				left_motor_set_speed(DRIVE_SPEED/2);
				while(get_object_det() == 2){
					chThdSleepMilliseconds(50);
				}
				break;
			case 3:
				//turn right slowly until object is found again
				right_motor_set_speed(-TURN_SPEED/2);
				left_motor_set_speed(TURN_SPEED/2);
				while(get_object_det() == 3){
					chThdSleepMilliseconds(50);
				}
				break;
			default:
				panic_handler("object_detection_value");
				return 0;
		}
	}while(get_object_det());
	return old_pos;
}

/*****************************Public Functions***********************************/

void turn_angle(int16_t angle){
	static int32_t angle_steps = 0;
	// reset position
	right_motor_set_pos(CLEAR);

	// Calculate angle in rotation of wheel in steps
	angle_steps = ONE_TURN_STEPS * angle / (2*PI_DEG);
	right_motor_set_speed(TURN_SPEED*sign(angle_steps));
	left_motor_set_speed(-TURN_SPEED*sign(angle_steps));

	do{ // turn right or left
		chThdSleepMilliseconds(50);
	}while((abs(angle_steps) > abs(right_motor_get_pos())));
}

//-----------------------------------------------------

void drive_distance(float distance){
	static float distance_steps = 0;
	// reset position
	right_motor_set_pos(CLEAR);

	// calculate distance in steps
	distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm
	right_motor_set_speed(DRIVE_SPEED);
	left_motor_set_speed(DRIVE_SPEED);

	do{
		chThdSleepMilliseconds(50);
	    if(get_object_det()){
	    	right_motor_set_pos(evade_obj_alg());
	    }
	}while((abs(distance_steps) > abs(right_motor_get_pos())));
}

//-------------------------------------------------------------------

void motor_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}
