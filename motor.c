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
#include <chprintf.h>

#define ONE_TURN_STEPS 1200
#define WHEEL_PERIM 13 //cm
#define TURN_SPEED 600
#define DRIVE_SPEED 800
#define EVADE_DISTANCE 4 //cm

static float diff_x = 0, diff_y = 0; // difference due to object in steps

/******************************Private Functions********************************/
int32_t evade_obj_alg(void){
	static int32_t old_pos = 0;
	static int32_t diff = 0;
	static int16_t alpha = 0;
	static float steps = 0; // float for sin calculations D:
	static int8_t sign_of_diff = 0;
	static int32_t evade_steps = 0;
	old_pos = right_motor_get_pos();
	motors_reset_pos();
	do{
		switch(get_object_det()){
			case 0:
				return old_pos; // definitely needs some calculations depending on the size of the object
				break;
			case 1:
				//turn left slowly
				motors_drive_dir(TURN_LEFT, 2);
				while(get_object_det() == 1){
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
					if(diff > ONE_TURN_STEPS){
						false_alarm(diff);
						break;
					}
				}
				steps = right_motor_get_pos(); // saves step count
				alpha = (diff / 2) * (PI_DEG / 2) / (ONE_TURN_STEPS / 4);  // -> angle turned
				break;

			case 2:
				//drive forward slowly until sensors lose object
				motors_drive_dir(FORWARDS, 2);
				while(get_object_det() == 2){
					chThdSleepMilliseconds(10);
				}
				// if we get to case 3 from 2 we drive a bit further
				if(get_object_det() == 3){
					// drive 4 cm so that the object is passed
					motors_drive_dir(FORWARDS, 1);
					evade_steps = EVADE_DISTANCE * ONE_TURN_STEPS / WHEEL_PERIM + right_motor_get_pos();
					while(evade_steps > right_motor_get_pos()){
						chThdSleepMilliseconds(10);
					}
				}
				steps = right_motor_get_pos() - steps; // subtract old count to get the distance
				diff_x += get_sin(alpha) * steps; // calculate difference in x
				diff_y += get_cos(alpha) * steps; // calculate difference in y
				break;

			case 3:
				//turn right slowly until object is found again
				motors_drive_dir(TURN_RIGHT, 2);
				sign_of_diff = sign(diff);
				while(get_object_det() == 3){
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos();
					if(sign_of_diff != sign(diff)){ // robot turned further right than initially -> passed simple object
						reset_obj_det();
					}
				}
				steps = right_motor_get_pos(); // saves step count
				diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
				alpha = (diff / 2) * (PI_DEG / 2) / (ONE_TURN_STEPS / 4);  // -> angle turned
				break;

			default:
				panic_handler("object_detection_value");
				return 0;
		}
	}while(get_object_det());
	steps = right_motor_get_pos() - steps; // subtract old count to get the distance
	diff_x += get_sin(alpha) * steps; // calculate difference in x
	diff_y += get_cos(alpha) * steps; // calculate difference in y

	motors_drive_dir(FORWARDS, 1);
	return old_pos;
}

/*****************************Public Functions***********************************/

void turn_angle(int16_t angle){
	static int32_t angle_steps = 0;
	// reset position
	motors_reset_pos();

	// Calculate angle in rotation of wheel in steps
	angle_steps = ONE_TURN_STEPS * angle / (2*PI_DEG);
	enum dir direction = TURN_LEFT;
	if(sign(angle_steps) < 0){
		direction = TURN_RIGHT;
	}
	motors_drive_dir(direction, 1);

	do{ // turn right or left
		chThdSleepMilliseconds(20);
	}while((abs(angle_steps) > abs(right_motor_get_pos())));
	motors_stop();
}

//-----------------------------------------------------

void drive_distance(float distance){
	static float distance_steps = 0;

	// calculate distance in steps
	distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm
	drive_steps(distance_steps, FORWARDS);

	// if an object was in the way we have a displacement in x and y to the goal
	if(diff_y != 0){
		// saved opposite to coord-system
		// +y means the robot is too far and needs to drive back a bit
		enum dir direction = BACKWARDS;
		if(diff_y < 0){
			direction = FORWARDS;
		}
		drive_steps((int)diff_y, direction);
	}

	// first, turn 90� degree to the right
	// drive forwards or backwards depending on the sign, +x means the robot is too
	// far left and because of the +90� turn needs to drive forward
	if(diff_x != 0){
		turn_angle(90);

		enum dir direction = FORWARDS;
		if(diff_y < 0){
			direction = BACKWARDS;
		}
		drive_steps((int)diff_x, direction);
	}
}

//------------------------------------------------------------------

void drive_steps(int32_t steps, enum dir direction){
	// reset position
	motors_reset_pos();

	// set speed + forwards or backwards
	motors_drive_dir(direction, 1);

	// wait and check every 20 milliseconds if goal is reached or object is detected
	do{
		chThdSleepMilliseconds(20);
	    if(get_object_det()){
	    	right_motor_set_pos(evade_obj_alg());
	    }
	}while((abs(steps) > abs(right_motor_get_pos())));
	motors_stop();
}

//-------------------------------------------------------------------

void motors_stop(void){
	right_motor_set_speed(OFF);
	left_motor_set_speed(OFF);
}

void motors_reset_pos(void){
	right_motor_set_pos(CLEAR);
	left_motor_set_pos(CLEAR);
}

void motors_drive_dir(enum dir direction, uint8_t fraction){
	if(fraction == 0){
		fraction = 1;
	}

	switch(direction){
		case FORWARDS:
			right_motor_set_speed(DRIVE_SPEED/fraction);
			left_motor_set_speed(DRIVE_SPEED/fraction);
			break;
		case BACKWARDS:
			right_motor_set_speed(-DRIVE_SPEED/fraction);
			left_motor_set_speed(-DRIVE_SPEED/fraction);
			break;
		case TURN_LEFT:
			right_motor_set_speed(TURN_SPEED/fraction);
			left_motor_set_speed(-TURN_SPEED/fraction);
			break;
		case TURN_RIGHT:
			right_motor_set_speed(-TURN_SPEED/fraction);
			left_motor_set_speed(TURN_SPEED/fraction);
			break;
		default:
			break;
	}
}
