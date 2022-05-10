/*
 * detection.c
 *
 *  Created on: 27.04.2022
 *      Author: Studium
 */
#include "detection.h"
#include "sensors/proximity.h"
#include <constants.h>
#include <leds.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chprintf.h>

static uint8_t object_detected = 0;

/****************************Private Functions***************************/

static THD_WORKING_AREA(waThdObstacleDetection, 128);
static THD_FUNCTION(ThdObstacleDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
/*
    //infinite loop
    while(1){
    	while(!object_detected){
    		// Test for object
    		if(get_calibrated_prox(FRONT_LEFT_IR_SENSOR) > IR_THRESHHOLD || get_calibrated_prox(FRONT_RIGHT_IR_SENSOR) > IR_THRESHHOLD){
    			object_detected = 1;
    		}
    		chThdSleepMilliseconds(50);
        	chprintf((BaseSequentialStream *)&SD3, "object detected = %d \r\n\n", object_detected);
    	}

    	while(object_detected){

    		switch(object_detected){
    			case 1:
    				// Turn left until right sensor "sees" the object
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) > IR_THRESHHOLD){
    					object_detected = 2;
    				}
    				break;
    			case 2:
    				// Drive straight until right sensor no longer detects an object
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) < IR_THRESHHOLD){
    					object_detected = 3;
    				}else if(get_calibrated_prox(RIGHT_FRONT_IR_SENSOR) > IR_THRESHHOLD){
    					object_detected = 1;
    				} // if not turned enough -> square object
    				break;
    			case 3:
    				// turn right until right sensor sees the object
    				if(get_calibrated_prox(RIGHT_IR_SENSOR) > IR_THRESHHOLD){
    					object_detected = 2;
    				}else if(get_calibrated_prox(RIGHT_FRONT_IR_SENSOR) > IR_THRESHHOLD){
    					object_detected = 1;
    				} // if it didn't catch right sensor while turning
    				break;
    			default:
    				break;
    		}
    		chThdSleepMilliseconds(10);
        	chprintf((BaseSequentialStream *)&SD3, "object detected = %d \r\n\n", object_detected);
    	}
    }

*/
    chThdSleepMilliseconds(10); //delete later
}

/*****************************Public Functions***********************************/

// creates thread in which the values of the IR sensor are processed
void obj_det_init(void){
	chThdCreateStatic(waThdObstacleDetection, sizeof(waThdObstacleDetection), NORMALPRIO, ThdObstacleDetection, NULL);
}

// returns object_detected which varies between 0-3
uint8_t get_object_det(void){
	return object_detected;
}

// sets object_detected back to 0 only if it's in phase 3 of the algorithm
void reset_obj_det(void){
	if(object_detected == 3){
		object_detected = 0;
	}
}

void false_alarm(int32_t diff){
	if(diff > FALSE_ALARM && object_detected == 1){
		object_detected = 3;
		// indication for us
		set_led(LED1, 1);
		set_led(LED3, 1);
		set_led(LED5, 1);
		set_led(LED7, 1);
		chThdSleepMilliseconds(300);
		clear_leds();
	}
}

