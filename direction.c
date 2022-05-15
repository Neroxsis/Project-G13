/*
 * direction.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include <direction.h>
#include <math.h>
#include <constants.h>
#include <sensors/imu.h>
#include <calculations.h>
#include <leds.h>

#include "msgbus/messagebus.h"

extern messagebus_t bus;

static float relative_rotation_z = 0;
static float x_acc_displacement = 0;
static float y_acc_displacement = 0;
static int16_t counter_displacement = 0;
static int16_t distance = 0;
static uint8_t picked_up = 0;

static enum state {pointA, displacement, pointB};
static enum state order = pointA;


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
    order = pointA;
    float period = PERIOD;

    while(1){ //infinite loop

    	// calculation relative_rotation (=angle by which robot was turned since reset)
    	time = chVTGetSystemTime();
		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

		// relative_rotation = rotation acceleration * period
		if (fabs(get_gyro_deg(&imu_values, Z_AXIS)) >= ROTATION_THRESHOLD){
			relative_rotation_z += get_gyro_deg(&imu_values, Z_AXIS) * period;
		}


		// ----------------------------------------------------------------------
		// Determine if robot is picked up or put down
		//     picked up: body LEDs on
		// on the ground: body LEDs off

		// as long as robot is not put down, check if it is being picked up
		// picked up: big acceleration in (z axis and (x or y axis))
		// 			  start measuring time
		if(order != pointB){
			// robot picked up
			if (((imu_values.acceleration[Z_AXIS]+GRAVITY) >= Z_ACC_THRESHOLD ||
					(imu_values.acceleration[Z_AXIS]+GRAVITY) <= -Z_ACC_THRESHOLD) &&
					(imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD ||
					imu_values.acceleration[X_AXIS] >= X_ACC_THRESHOLD)){
				picked_up = IN_AIR;
				if(order == pointA){
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
				if(order == displacement){
					time_of_flight = chVTGetSystemTime() - time_of_flight; // duration of flight in System ticks
					distance = ST2MS(time_of_flight)/10 * SPEED_MMPCS;
				}
				set_body_led(picked_up);
			}

			// if robot picked up: point A -> displacement
			// if robot NOT picked up: displacement -> pointB
			if (picked_up){
				order = displacement;
			} else {
				if (order == displacement){
					order = pointB;
				}
			}
		}

		// values to calculate return angle are saved here
		// x_acc and y_acc saved 360ms after robot was picked up
		if(order == displacement){
		   	counter_displacement ++;
		   	if(counter_displacement == MAX_COUNTER_DISPLACEMENT){
		   		x_acc_displacement = imu_values.acceleration[X_AXIS];
		   		y_acc_displacement = imu_values.acceleration[Y_AXIS];
		   	}
		}

		// IMU reads new values every 250 Hz
		 chThdSleepUntilWindowed(time, time + MS2ST(PERIOD*100));  //magic number?

    }
}

void direction_init(void){
	chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
}

int16_t get_distance(void){
	return distance;
}

void set_order(int8_t i){
	if (i==POINTA_INT){order=pointA;}
	if (i==DISPLACEMENT_INT){order=displacement;}
	if (i==POINTB_INT){order=pointB;}
}

int8_t check_order_pointB(void){
	if(order == pointB){
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

// input: x_acc, y_acc measured shortly after robot picked up
//		  angle: relative rotation of robot measured at pointB
//output: degrees that robot has to turn to face the direction of pointA
float return_angle(float x_acc, float y_acc, float angle){
	int16_t theta = CLEAR;

	// - PI <= theta <= PI
	theta = (int16_t)(atan2(y_acc, x_acc)*PI_DEG/M_PI);

	// error management:
	// if robot moved less than 30mm, return angle = 0
	if (distance <= DISTANCE_THRESHOLD){
		return 0;
	}
	// Case 1: Robot is moved along an axis
	// robot moved along y axis -> very small x_acc
	if(fabs(x_acc) <= ACC_THRESHOLD){
		if(y_acc > 0){
			return - angle;		// positive y axis
		}else{
			return PI_DEG_F - angle;	// negative y axis
		}

	// robot moved along x axis -> very small y_acc
	}else if(fabs(y_acc) <= ACC_THRESHOLD){
		if(x_acc > 0){
			return -PI_DEG_F/2 - angle;	//positive x_axis
		}else{
			return PI_DEG_F/2 - angle;	//negative x_axis
		}

	// Case 2: Robot is NOT moved along an axis
	}else{
		//divide into quadrants using sign of x_acc, y_acc
		if(x_acc > 0){
			if(y_acc > 0){
				//quadrant I
				return - PI_DEG_F/2 + theta - angle;
			} else {
				//quadrant IV
				return - PI_DEG_F - theta + angle;
			}
		}

		if(x_acc < 0){
			if(y_acc > 0){
				//quadrant II  OK
				return - PI_DEG_F/2 + theta - angle;
			} else {
				//quadrant III
				return PI_DEG_F/2 + (PI_DEG_F + theta) - angle;
			}
		}
	}

	return 0;

}

int16_t get_angle(void){
	return return_angle(x_acc_displacement, y_acc_displacement, relative_rotation_z);
}

void set_leds1357(int8_t i){
	set_led(LED1, i);
	chThdSleepMilliseconds(SLEEP_200);
	set_led(LED3, i);
	chThdSleepMilliseconds(SLEEP_200);
	set_led(LED5, i);
	chThdSleepMilliseconds(SLEEP_200);
	set_led(LED7, i);
	chThdSleepMilliseconds(SLEEP_200);
}
