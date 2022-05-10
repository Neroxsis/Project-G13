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

#include <chprintf.h>

#include <main.h>
#include "leds.h"
#include "motor.h"



extern messagebus_t bus;


static imu_msg_t imu_values;

static uint8_t picked_up = 0;
static float distance = 0; // mm



static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



    systime_t time;

    imu_msg_t imu_values;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");  //vlt als parameter mitgeben; in air definition HIER !!!




    // variable declarations
    float x_speed = 0;  // m/s
    float y_speed = 0;  // m/s
    float x_position = 0; // mm
    float y_position = 0; // mm
    float period = 0.1;

    static int16_t relative_rotation = 0;
    static uint8_t goal_reached = FALSE;
    static int8_t in_air = FALSE;


#define TRUE 1
#define FALSE 0
#define Z_ACC_THRESHOLD 0.08
#define GRAVITY 9.81
#define X_ACC_THRESHOLD 0.08
#define Y_ACC_THRESHOLD 0.08
#define X_SPEED_THRESHOLD 0.08
#define Y_SPEED_THRESHOLD 0.08



    //updates imu values
    messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

	//check if robot is on the table or in the air
	if (fabs(imu_values.acceleration[Z_AXIS]+GRAVITY) <= Z_ACC_THRESHOLD){
		in_air = FALSE;
	} else {
		in_air = TRUE;
	}
	chprintf((BaseSequentialStream *)&SD3, " z_acc  =%.2f \r\n\n", imu_values.acceleration[Z_AXIS]+GRAVITY);
	chprintf((BaseSequentialStream *)&SD3, " in_air  =%d \r\n\n", in_air);

//  !!!  it prints those 2 lines only twice ... WEIRD

//  !!! only enters the while loop once
    	while(1){ //while(in_air){


    	//updates imu values
    	//messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));



    	drive_distance(100);
/*
// --------------------------------------------------------
    	chprintf((BaseSequentialStream *)&SD3, "%__________________________________________________________ \r\n\n");

    	// get angle from IMU gyroscope
    	// get_gyro_deg gives you deg/s -> we need to multiply it by a period to get the actual angle in deg
    	chprintf((BaseSequentialStream *)&SD3, "%get_gyro_deg, Z =%d \r\n\n", get_gyro_deg(&imu_values, Z_AXIS));
    	// calculate angle
    	relative_rotation += get_gyro_deg(&imu_values, Z_AXIS) * period;
    	chprintf((BaseSequentialStream *)&SD3, "%relative_rotation, Z =%d \r\n\n", relative_rotation);



    	chprintf((BaseSequentialStream *)&SD3, "Calculate speed \r\n\n");
		chprintf((BaseSequentialStream *)&SD3, " x_acc =%.2f, y_acc =%.2f \r\n\n", imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS]);


		// multiply by corrective factor of 100
		if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD && fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
			x_speed += 100 * period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
			y_speed += 100 * period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));
		}

		chprintf((BaseSequentialStream *)&SD3, " x_speed  =%.2f \r\n\n", x_speed);
    	chprintf((BaseSequentialStream *)&SD3, " y_speed  =%.2f \r\n\n", y_speed);

// --------------------------------------------------------

    	if (!in_air){
    	    x_speed=0;
    	    y_speed=0;
    	    chprintf((BaseSequentialStream *)&SD3, "set speeds to 0");

    	    drive_distance(1000);  // for testing purposes

    	}

// --------------------------------------------------------

    	x_position += period * x_speed;
    	y_position += period * y_speed;

    	chprintf((BaseSequentialStream *)&SD3, " x_position=%.2f y_position=%.2f\r\n\n",  x_position, y_position);


    	distance = sqrt(x_position*x_position + y_position*y_position);

    	chprintf((BaseSequentialStream *)&SD3, " distance=%.2f \r\n\n", distance);

*/

    	chThdSleepMilliseconds(10);
    }


}


int8_t get_in_air(void){
	if(in_air == true){
		return 1;
	}else{
		return 0;
	}
}


float get_distance(void){
	return distance;
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


// maybe increase priority by 1 ?!?
void goal_calc_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}
