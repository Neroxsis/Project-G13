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

static imu_msg_t imu_values;
static uint8_t picked_up = 0;

static THD_WORKING_AREA(waThdGoalCalculations, 128);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    imu_msg_t imu_values;

    uint16_t no_mvmt_counter = 0;

    float x_speed = 0;  // mm/s
    float y_speed = 0;  // mm/s

    float x_position = 0;
    float y_position = 0;

    int16_t relative_rotation = 0;
    int16_t rotation_speed =0;
    int16_t theta = 0; 		// theta = arctan(y/x) to calculate angle_to_goal

    float period = 0; // because float * float faster than float * int
    uint16_t period_t = 0; // for angle calculations

    int16_t angle_to_goal = 0;
    int32_t distance_to_goal = 0;

#define ACCELERATION_NOISE_TH 42
#define NINETY_DEGREE 90
#define WAIT_TIME 1234
#define ACTIVATION_TH 45245

    uint8_t in_air = 0;

    while(!in_air){
    	if(imu_values.acceleration[Z_AXIS] > ACTIVATION_TH){
    		in_air = 1;
    	} // add sleep
    }

    while(in_air){
    	// wait for measurements or a defined time
    	//messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	// or chThdSleepMilliseconds(10);

    	// get acceleration measurements into imu_values
    	//also messagebus_topic_wait

    	// calculate angle
    	rotation_speed += get_gyro_deg(&imu_values, Z_AXIS) * period_t; // deg/s
    	relative_rotation += rotation_speed * period_t;

    	// calculate speed
    	x_speed += period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
    	y_speed += period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));

    	// calculate distance
    	x_position += period * x_speed;
    	y_position += period * y_speed;

    	// test if epuck still in air
    	if(1){
    		no_mvmt_counter++;
    		if(no_mvmt_counter > WAIT_TIME){
    			in_air = 0;
    		}
    	}else{
    		no_mvmt_counter = 0;
    	}
    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

uint8_t in_air(void){
	return picked_up;
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



