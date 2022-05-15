/*
 * detection.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include "detection.h"
#include "sensors/proximity.h"
#include <constants.h>
#include <leds.h>

static uint8_t object_detected = 0;
static uint8_t detect = 0;

/****************************Private Functions***************************/

static THD_WORKING_AREA(waThdObstacleDetection, 128);
static THD_FUNCTION(ThdObstacleDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    //infinite loop
    while(1){
    	while(!object_detected && detect){
    		// Test for object
    		if(get_calibrated_prox(FRONT_LEFT_IR_SENSOR) > IR_THRESHOLD || get_calibrated_prox(FRONT_RIGHT_IR_SENSOR) > IR_THRESHOLD){
    			object_detected = EVADE_TURN_LEFT;
    		}
    		chThdSleepMilliseconds(20); //a bit slower than 50 Hz should be enough
    	}

    	while(object_detected){
    		time = chVTGetSystemTime();
    		switch(object_detected){
    			case EVADE_TURN_LEFT:
    				// Turn left until right sensor "sees" the object and 45° front no longer sees it
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) > IR_THRESHOLD && get_calibrated_prox(RIGHT_FRONT_IR_SENSOR)){
    					object_detected = EVADE_DRIVE_STRAIGHT;
    				}
    				break;
    			case EVADE_DRIVE_STRAIGHT:
    				// Drive straight until right sensor no longer detects an object
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) < IR_THRESHOLD){
    					object_detected = EVADE_TURN_RIGHT;
    				}else if(get_calibrated_prox(RIGHT_FRONT_IR_SENSOR) > IR_THRESHOLD){
    					object_detected = EVADE_TURN_LEFT;
    				} // if not turned enough -> square object
    				break;
    			case EVADE_TURN_RIGHT:
    				// turn right until right sensor sees the object
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) > IR_THRESHOLD){
    					object_detected = EVADE_DRIVE_STRAIGHT;
    				}else if(get_calibrated_prox(RIGHT_FRONT_IR_SENSOR) > IR_THRESHOLD){
    					object_detected = EVADE_TURN_LEFT;
    				} // if it didn't catch right sensor while turning
    				break;
    			default:
    				break;
    		}
    		chThdSleepUntilWindowed(time, time + MS2ST(10)); // 100 Hz
    	}
    }
}

/*****************************Public Functions***********************************/

// creates thread in which the values of the IR sensor are processed
void obj_det_init(void){
	chThdCreateStatic(waThdObstacleDetection, sizeof(waThdObstacleDetection), NORMALPRIO, ThdObstacleDetection, NULL);
}

//-------------------------------------------------------------------------

// returns object_detected which varies between 0-4 used by motor object evade algo
uint8_t get_object_det(void){
	return object_detected;
}

//-------------------------------------------------------------------------

// sets object_detected back to 0 only if it's in phase 3 or 4 (false alarm) of the algorithm
void reset_obj_det(void){
	if(object_detected == EVADE_TURN_RIGHT || object_detected == EVADE_FA){
		object_detected = CLEAR;
	}
}

//-------------------------------------------------------------------------

void start_detection(void){
	detect = ON;
}

void end_detection(void){
	detect = OFF;
}

//-----------------------------------------------------------------

// set object_detected to 4 so that the motor turns back, if it is a false alarm
void false_alarm(int32_t diff){
	if(diff > FALSE_ALARM && object_detected == EVADE_TURN_LEFT){
		object_detected = EVADE_FA;
		// indication for us
		set_led(LED1, ON);
		set_led(LED3, ON);
		set_led(LED5, ON);
		set_led(LED7, ON);
		chThdSleepMilliseconds(300);
		clear_leds();
	}
}

