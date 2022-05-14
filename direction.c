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


static float relative_rotation_z = 0;
static int16_t distance = 0;


static float x_acc_displacement = 0;
static float y_acc_displacement = 0;

static uint8_t picked_up = 0;
static int16_t counter_displacement = 0;

static enum state {pointA, displacement, pointB};
static enum state order = pointA;

static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    systime_t time_of_flight = chVTGetSystemTime();
    imu_msg_t imu_values;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    order = pointA;
    int8_t counter_small_acc = 0;
    float period = 0.012;

    while(1){ //infinite loop

    	// calculation relative_rotation
    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		if (fabs(get_gyro_deg(&imu_values, Z_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_z += get_gyro_deg(&imu_values, Z_AXIS) * period;
		}



		// ----------------------------------------------------------------------
		//	Determines if robot is picked up and then turns on body lights
		// doesn't work anymore if u delete the set body led in the for loops


		 // pointB if robot is put down
		 if(order != pointB){
			 //pick up
			 if (((imu_values.acceleration[Z_AXIS]+GRAVITY) >= Z_ACC_THRESHOLD ||
					(imu_values.acceleration[Z_AXIS]+GRAVITY) <= -Z_ACC_THRESHOLD) &&
					(imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD ||
						imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD)){
				picked_up = 1;
				if(order == pointA){
					time_of_flight = chVTGetSystemTime(); // save time of departure
				}
				set_body_led(picked_up);
			 }

			// put down
			if (fabs(imu_values.acceleration[X_AXIS]) <= X_ACC_THRESHOLD &&
				fabs(imu_values.acceleration[Y_AXIS]) <= Y_ACC_THRESHOLD){
				picked_up = 0;
				if(order == displacement){
					time_of_flight = chVTGetSystemTime() - time_of_flight; // duration of flight in System ticks
					distance = ST2MS(time_of_flight)/10 * SPEED_MMPCS;
				}
				set_body_led(picked_up);
				set_picked_up(picked_up);
			}

			// pointB if robot is put down

			if (picked_up){
				order = displacement;
			} else {
				if (order == displacement){
					order = pointB;
				}
			}
		}

		//return angle is calculated here
		if(order == displacement){
		   	counter_displacement ++;
		   	if(counter_displacement == 30){
		   		x_acc_displacement = imu_values.acceleration[X_AXIS];
		   		y_acc_displacement = imu_values.acceleration[Y_AXIS];
		   	}
		}//end if

		 chThdSleepUntilWindowed(time, time + MS2ST(12));	// IMU reads new values every 250 Hz -> Source CITE !!!

    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

float get_relative_rotation_z(void){
	return relative_rotation_z;
}

void set_relative_rotation_z(float rotation){
	relative_rotation_z = rotation;
}

int16_t get_distance(void){
	return distance;
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



enum state get_order(void){
	return order;
}

void set_order(int8_t i){
	if (i==0){order=pointA;}
	if (i==1){order=displacement;}
	if (i==2){order=pointB;}
}

int8_t check_order_pointB(void){
	if(order == pointB){
		return 1;
	}else{
		return 0;
	}
}

void reset_direction(void){
	counter_displacement = 0;
	x_acc_displacement = 0;
	relative_rotation_z = 0;
}



// phi = relative_rotation_z	-> does NOT work yet
//or I could just take the acc during the displacement and then take the sign here...
//rethink the stuff I'm giving as parameters ... !!
float return_angle(float x_acc, float y_acc, float angle){
	int16_t theta = 0;
	float THRESHOLD = 0.08; //still has to be calibrated

	theta = (int16_t)(atan2(y_acc, x_acc)*PI_DEG/M_PI);

	//if distance << -> skip this step

	if(fabs(x_acc) <= THRESHOLD){
		if(y_acc > 0){
			return 0.0 - angle;
		}else{
			return 180.0 - angle;
		}
	}else if(fabs(y_acc) <= THRESHOLD){
		if(x_acc > 0){
			return -90.0 - angle;
		}else{
			return 90.0 - angle;
		}
	}else{

		if(x_acc > 0){
			if(y_acc > 0){	// quadrant I  OK
				return -90 + theta - angle;
			} else {							// quadrant IV
				return - 180 - theta + angle;
			}
		}

		if(x_acc < 0){
			if(y_acc > 0){	// quadrant II  OK
				return -90 + theta - angle;
			} else {							// quadrant III
				return 90 + (180 + theta) - angle;
			}
		}
	}

	return 0;

}


