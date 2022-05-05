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
#define FORWARDS 1
#define BACKWARDS -1

static float diff_x = 0, diff_y = 0; // difference due to object in steps

/******************************Private Functions********************************/
int32_t evade_obj_alg(void){
	static int32_t old_pos = 0;
	static int32_t diff = 0;
	static int16_t alpha = 0;
	static float steps = 0; // for sin calculations D:
	static int8_t sign_of_diff = 0;
	old_pos = right_motor_get_pos();
	motors_reset_pos();
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
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
					if(diff > 1000){
						false_alarm(diff);
						break;
					}
				}
				steps = right_motor_get_pos(); // saves step count
				alpha = diff * (PI_DEG / 2) / (ONE_TURN_STEPS / 4);  // -> angle turned
				break;
			case 2:
				//drive forward slowly until sensors lose object
				right_motor_set_speed(DRIVE_SPEED/2);
				left_motor_set_speed(DRIVE_SPEED/2);
				while(get_object_det() == 2){
					chThdSleepMilliseconds(10);
				}
				steps = right_motor_get_pos() - steps; // subtract old count to get the distance
				diff_x += get_sin(alpha) * steps; // calculate difference in x
				diff_y += get_cos(alpha) * steps; // calculate difference in y
				break;
			case 3:
				//turn right slowly until object is found again
				right_motor_set_speed(-TURN_SPEED/2);
				left_motor_set_speed(TURN_SPEED/2);
				sign_of_diff = sign(diff);
				while(get_object_det() == 3){
					chThdSleepMilliseconds(50);
					diff = right_motor_get_pos() - left_motor_get_pos();
					if(sign_of_diff != sign(diff)){ // robot turned further right than initially -> passed simple object
						reset_obj_det();
					}
				}
				steps = right_motor_get_pos(); // saves step count
				diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
				alpha = diff * (PI_DEG / 2) / (ONE_TURN_STEPS / 4);  // -> angle turned
				break;
			default:
				panic_handler("object_detection_value");
				return 0;
		}
	}while(get_object_det());
	steps = right_motor_get_pos() - steps; // subtract old count to get the distance
	diff_x += get_sin(alpha) * steps; // calculate difference in x
	diff_y += get_cos(alpha) * steps; // calculate difference in y

	right_motor_set_speed(DRIVE_SPEED);
	left_motor_set_speed(DRIVE_SPEED);
	return old_pos;
}

/*****************************Public Functions***********************************/

void turn_angle(int16_t angle){
	static int32_t angle_steps = 0;
	// reset position
	motors_reset_pos();

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

	// calculate distance in steps
	distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm

	drive_steps(distance_steps, FORWARDS);

	// if an object was in the way we have a difference in x and y
	// which needs to be corrected
	if(diff_y != 0){
		// saved opposite to coord-system
		// +y means the robot is too far and needs to drive back a bit
		drive_steps((int)diff_y, -signf(diff_y));
	}

	if(diff_x != 0){
		// first, turn 90° degree to the right
		turn_angle(90);
		// drive forwards or backwards depending on the sign
		// +x means the robot is too low and because
		// of the +90° turn needs to drive forward
		drive_steps((int)diff_x, signf(diff_x));
	}
}

//------------------------------------------------------------------

// direction positive means forwards, negative means backwards,
// 0 will be taken as forwards
void drive_steps(int32_t steps, int8_t direction){
	// correct a 0 in direction
	if(direction == 0){
		direction = 1;
	}

	// reset position
	motors_reset_pos();

	// set speed + forwards or backwards
	right_motor_set_speed(DRIVE_SPEED * signs(direction));
	left_motor_set_speed(DRIVE_SPEED * signs(direction));

	// wait and check every 50 milliseconds if goal is reached or object is detected
	do{
		chThdSleepMilliseconds(50);
	    if(get_object_det()){
	    	right_motor_set_pos(evade_obj_alg());
	    }
	}while((abs(steps) > abs(right_motor_get_pos())));
}

//-------------------------------------------------------------------

void motor_stop(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void motors_reset_pos(void){
	right_motor_set_pos(CLEAR);
	left_motor_set_pos(CLEAR);
}
