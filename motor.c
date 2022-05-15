/*
 * motor.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include <arm_math.h>
#include <stdlib.h>
#include <constants.h>
#include <calculations.h>
#include <motor.h>
#include <detection.h>

#include "motors.h"

static float diff_x = 0; // difference due to object in steps
static uint8_t found_goal = 0;

/**************************************Private Functions*******************************************/

// algorithm to evade an object
int32_t evade_obj_alg(void){
	static int32_t old_pos = 0;
	static int32_t diff = 0;
	static int16_t alpha = 0;
	static float diff_y = 0; // difference due to object in steps
	static float steps = 0; // float for sin calculations
	static int8_t sign_of_diff = 0;
	static int32_t evade_steps = 0;
	old_pos = right_motor_get_pos(); // save old step count
	motors_reset_pos();
	do{
		switch(get_object_det()){
			case 0:
				break;
			case EVADE_TURN_LEFT:
				//turn left slowly
				motors_drive_dir(TURN_LEFT, SLOW);
				while(get_object_det() == EVADE_TURN_LEFT){
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
					if(diff > ONE_TURN_STEPS){ // if the robot turned too far left
						false_alarm(diff);
						break;
					}
				}
				steps = right_motor_get_pos(); // saves step count
				alpha = (diff / 2) * PI_DEG / (ONE_TURN_STEPS / 2);  // -> angle turned
				break;

			case EVADE_DRIVE_STRAIGHT:
				//drive forward slowly until sensors lose object
				motors_drive_dir(FORWARDS, SLOW);
				while(get_object_det() == EVADE_DRIVE_STRAIGHT){
					chThdSleepMilliseconds(10);
				}
				// if we go to case 3 from 2 we drive a bit further
				if(get_object_det() == EVADE_TURN_RIGHT){
					// drive 4 cm so that the object is passed
					motors_drive_dir(FORWARDS, FAST);
					evade_steps = EVADE_DISTANCE * ONE_TURN_STEPS / WHEEL_PERIM + right_motor_get_pos();
					while(evade_steps > right_motor_get_pos()){
						chThdSleepMilliseconds(10);
					}
				}
				steps = right_motor_get_pos() - steps; // subtract old count to get the distance
				diff_x += get_sin(alpha) * steps; // calculate difference in x
				diff_y += get_cos(alpha) * steps; // calculate difference in y
				break;

			case EVADE_TURN_RIGHT:
				//turn right slowly until object is found again
				motors_drive_dir(TURN_RIGHT, SLOW);
				sign_of_diff = sign(diff);
				while(get_object_det() == EVADE_TURN_RIGHT){
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos();
					if(sign_of_diff != sign(diff)){ // robot turned further right than initially -> passed simple object
						reset_obj_det();
					}
				}
				steps = right_motor_get_pos(); // saves step count
				diff = right_motor_get_pos() - left_motor_get_pos(); // calculates difference between right and left wheel
				alpha = (diff / 2) * PI_DEG / (ONE_TURN_STEPS / 2);  // -> angle turned
				break;

			case EVADE_FA:
				// false alarm
				motors_drive_dir(TURN_RIGHT, SLOW);
				sign_of_diff = sign(diff);
				while(get_object_det() == EVADE_FA){
					chThdSleepMilliseconds(10);
					diff = right_motor_get_pos() - left_motor_get_pos();
					if(sign_of_diff != sign(diff)){
						reset_obj_det();
					}
				}
				break;

			default:
				panic_handler("object_detection_value");
				return 0;
		}
	}while(get_object_det());
	motors_drive_dir(FORWARDS, FAST);
	old_pos += diff_y; // add distance traveled while evading object to old step count
	diff_y = CLEAR; // reset
	return old_pos;
}

/*****************************Public Functions***********************************/

// turn the robot by a given angle. positive angle -> left turn, negative -> right turn
void turn_angle(int16_t angle){
	static int32_t angle_steps = 0;
	// reset position
	motors_reset_pos();

	// Calculate angle in rotation of wheel in steps
	angle_steps = ONE_TURN_STEPS * angle / (2*PI_DEG);
	// define direction
	enum dir direction = TURN_LEFT;
	if(angle_steps < 0){
		direction = TURN_RIGHT;
	}
	motors_drive_dir(direction, FAST);

	while((abs(angle_steps) > abs(right_motor_get_pos())) && !found_goal){
		chThdSleepMilliseconds(20);
	}
	//motors_stop();
}

//-----------------------------------------------------

// drive a given distance, while evading objects. distance is in mm
void drive_distance(float distance){
	static float distance_steps = 0;

	// calculate distance in steps
	distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm
	drive_steps(distance_steps, FORWARDS);

	// first, turn 90° degree clockwise
	// drive forwards or backwards depending on the sign, +x means the robot is too
	// far left and because of the +90° turn needs to drive forward
	if(diff_x){
		turn_angle(-90);

		enum dir direction = FORWARDS;
		if(diff_x < 0){
			direction = BACKWARDS;
		}
		drive_steps((int)diff_x, direction);
		diff_x = CLEAR; // reset
	}
	motors_stop();

	// if the robot found the goal platform
	if(found_goal){
		found_goal = CLEAR; // reset
		distance_steps = DIST_TO_GOAL * ONE_TURN_STEPS / WHEEL_PERIM;
		drive_steps(distance_steps, FORWARDS);
	}else{ // if not turn on spot
		turn_angle(360);
		if(found_goal){
			found_goal = CLEAR; // reset
			distance_steps = DIST_TO_GOAL * ONE_TURN_STEPS / WHEEL_PERIM;
			drive_steps(distance_steps, FORWARDS);
		}
	}
	motors_stop();
}

//------------------------------------------------------------------

// dive a distance forwards or backwards. distance in steps
void drive_steps(int32_t steps, enum dir direction){
	// reset position
	motors_reset_pos();

	// set speed + forwards or backwards
	motors_drive_dir(direction, FAST);

	// wait and check every 20 milliseconds if goal is reached or object is detected
	while(abs(steps) > abs(right_motor_get_pos()) && !found_goal){
		chThdSleepMilliseconds(20);
	    if(get_object_det() && !found_goal){
	    	right_motor_set_pos(evade_obj_alg());
	    }
	}
	motors_stop();
}

//-------------------------------------------------------------------

void motors_stop(void){
	right_motor_set_speed(OFF);
	left_motor_set_speed(OFF);
}

//-------------------------------------------------------------------

void motors_reset_pos(void){
	right_motor_set_pos(CLEAR);
	left_motor_set_pos(CLEAR);
}

//-------------------------------------------------------------------

// set the motor speed to be able to drive forwards, backwards or turn on the spot
// fraction sets how many fractions of the max_speed the movement is
void motors_drive_dir(enum dir direction, uint8_t fraction){
	// if fraction = 0 -> fraction = 1;
	if(!fraction){
		fraction = FAST;
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

//-------------------------------------------------------

// used to tell that the red paper was found
void set_found_goal(void){
	found_goal = ON;
}
