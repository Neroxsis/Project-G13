/*
 * direction.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include <direction.h>
#include <arm_math.h>
#include <stdlib.h>
#include <constants.h>
#include <sensors/imu.h>
#include <calculations.h>
#include <leds.h>

#include "msgbus/messagebus.h"

extern messagebus_t bus;

static float x_acc_displacement = 0;
static float y_acc_displacement = 0;
static int16_t relative_rotation_z = 0;
static int16_t counter_displacement = 0;
static int16_t distance = 0;
static uint8_t picked_up = 0;

static enum states state = pointA;


// calculates by which angle the robot has to turn to go to pointA
static THD_WORKING_AREA(waThdGoalCalculations, 1024);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    systime_t time_of_flight = chVTGetSystemTime();
    imu_msg_t imu_values;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    // robot starts at pointA
    state = pointA;

    while(1){ //infinite loop

    	// calculation relative_rotation (=angle by which robot was turned since reset)
    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		// relative_rotation = rotation acceleration * PERIOD
		if (abs(get_gyro_deg(&imu_values, Z_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_z += get_gyro_deg(&imu_values, Z_AXIS) * PERIOD;
		}


		// ----------------------------------------------------------------------
		// Determine if robot is picked up or put down
		//     picked up: body LEDs on
		// on the ground: body LEDs off

		// as long as robot is not put down, check if it is being picked up
		// picked up: big acceleration in (z axis and (x or y axis))
		// 			  start measuring time
		if(state != pointB){
			// robot picked up
			if (((imu_values.acceleration[Z_AXIS]+GRAVITY) >= Z_ACC_THRESHOLD ||
					(imu_values.acceleration[Z_AXIS]+GRAVITY) <= -Z_ACC_THRESHOLD) &&
					(imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD ||
					imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD)){
				picked_up = IN_AIR;
				if(state == pointA){
					time_of_flight = chVTGetSystemTime(); // save time of departure
				}
				set_body_led(picked_up);
			 }

			// robot put down
			// not picked up: x_acc and y_acc are small
			// calculate duration of flight
			if (fabs(imu_values.acceleration[X_AXIS]) <= X_ACC_THRESHOLD &&
				fabs(imu_values.acceleration[Y_AXIS]) <= Y_ACC_THRESHOLD){
				picked_up = ON_GROUND;
				if(state == displacement){
					time_of_flight = chVTGetSystemTime() - time_of_flight; // duration of flight in System ticks
					distance = ST2MS(time_of_flight)/10 * SPEED_MMPCS;	// div by 10: convertion ms to cs
				}
				set_body_led(picked_up);
			}

			// if robot picked up: point A -> displacement
			// if robot NOT picked up: displacement -> pointB
			if (picked_up){
				state = displacement;
			} else {
				if (state == displacement){
					state = pointB;
				}
			}
		}

		// values to calculate return angle are saved here
		// x_acc and y_acc saved 360ms after robot was picked up
		if(state == displacement){
		   	counter_displacement ++;
		   	if(counter_displacement == MAX_COUNTER_DISPLACEMENT){
		   		x_acc_displacement = imu_values.acceleration[X_AXIS];
		   		y_acc_displacement = imu_values.acceleration[Y_AXIS];
		   	}
		}

		// IMU reads new values every 250 Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(PERIOD*100)); //*100: convert 0.012s to 12ms

    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

int16_t get_distance(void){
	return distance;
}

void set_state(enum states state_){
	state = state_;
}

int8_t check_state_pointB(void){
	if(state == pointB){
		return 1;
	}else{
		return 0;
	}
}

void reset_direction(void){
	counter_displacement = CLEAR;
	x_acc_displacement = CLEAR;
	y_acc_displacement = CLEAR;
	relative_rotation_z = CLEAR;
}

int16_t get_angle(void){
	// error management:
	// if robot moved less than 20mm, return angle = 0
	if (get_distance() <= DISTANCE_THRESHOLD){
		return 0;
	} else {
		return return_angle(x_acc_displacement, y_acc_displacement, relative_rotation_z);
	}
}

void set_leds1357(int8_t i){
	set_led(LED1, i);
	chThdSleepMilliseconds(200);
	set_led(LED3, i);
	chThdSleepMilliseconds(200);
	set_led(LED5, i);
	chThdSleepMilliseconds(200);
	set_led(LED7, i);
	chThdSleepMilliseconds(200);
}
