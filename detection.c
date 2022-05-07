/*
 * detection.c
 *
 *  Created on: 27.04.2022
 *      Author: Studium
 */
#include "detection.h"
#include "sensors/proximity.h"
#include <constants.h>


static uint8_t object_detected = 0;

/****************************Private Functions***************************/

static THD_WORKING_AREA(waThdObstacleDetection, 128);
static THD_FUNCTION(ThdObstacleDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(!object_detected){
    	// Test for object
    	if(get_calibrated_prox(FRONT_LEFT_IR_SENSOR) > TH && get_calibrated_prox(FRONT_RIGHT_IR_SENSOR) > TH){
    		object_detected = 1;
    	}
    	// Sleep
    }

    while(object_detected){

    	switch(object_detected){
    		case 1:
    			// Turn left until right sensor "sees" the object
    			if(get_calibrated_prox(RIGHT_IR_SENSOR) > TH){
    				object_detected = 2;
    			}
    		case 2:
    			// Read right or left sensor maybe the two further back as well
    			// If no sensor detects an object set object_detected to 3
    			if(get_calibrated_prox(RIGHT_IR_SENSOR) < TH && get_calibrated_prox(RIGHT_BACK_IR_SENSOR) > TH){
    				object_detected = 3;
    			}
    			break;
    		case 3:
    			// turn right until 45� right front sensor sees the object
    			if(get_calibrated_prox(RIGHT_FRONT_IR_SENSOR) > TH){
    				object_detected = 1;
    			}
    			break;
    			// needs further algorithms to set object_detected to 0
    			// now it just circles around the object
    		default:
    			break;
    	}
    }
}

/*****************************Public Functions***********************************/

void obj_det_init(void){
	chThdCreateStatic(waThdObstacleDetection, sizeof(waThdObstacleDetection), NORMALPRIO, ThdObstacleDetection, NULL);
}

uint8_t get_object_det(void){
	return object_detected;
}
