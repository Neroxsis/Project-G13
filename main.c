#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include "sensors/proximity.h"
#include "leds.h"

#include <arm_math.h>

#include <constants.h>
#include "calculations.h"
#include <detection.h>
#include "motor.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

uint8_t no_mvmt_detected(imu_msg_t* imu_values);

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
    	if(no_mvmt_detected(&imu_values)){
    		no_mvmt_counter++;
    		if(no_mvmt_counter > WAIT_TIME){
    			in_air = 0;
    		}
    	}else{
    		no_mvmt_counter = 0;
    	}
    }
}

uint8_t no_mvmt_detected(imu_msg_t* imu_values){
#define THRESHHOLD 42
#define GYRO_THRESHHOLD 2
	float x_speed=0, y_speed=0, rotation_speed=0; // you need to declare or pass them via struct for example
	if(x_speed < THRESHHOLD &&
		y_speed < THRESHHOLD &&
		rotation_speed < THRESHHOLD ){
		return 1;
	}else{
		return 0;
	}
}

// sets and returns 1 if "true", 0 if "false" and returns the case if given value is anything else, i.e., invalid -> request state

uint8_t in_air(uint8_t tf){
	static uint8_t in_air = 0;
	switch(tf){
		case 0:
			in_air = 0;
			return in_air;
			break;
		case 1:
			in_air = 1;
			return in_air;
			break;
		default:
			return in_air;
	}
}
int32_t evade_obj_alg(void); // just temporary to build
//------------------------------------------------------------------------

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    serial_start();
    usb_start();
    motors_init();
    imu_start();
    proximity_start();
    clear_leds();
    set_body_led(0);
    set_front_led(0);
    lookup_init();
    obj_det_init();

    //start calibration
    //indication that calibration is in progress
    set_led(LED1, 1);
    set_led(LED3, 1);
    set_led(LED5, 1);
    set_led(LED7, 1);

    calibrate_gyro();
    set_led(LED1, 0);
    calibrate_acc();
    set_led(LED3, 0);
    calibrate_ir();
    set_led(LED5, 0);
    set_led(LED7, 0);

    //create threads
    chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
    //messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    //infinite loop
    while(1){
    	static uint8_t goal_reached = 0;
    	static int16_t relative_rotation = 0;
    	static float distance = 0; // mm

    	while(!in_air(GET) && !goal_reached){
    	    // First turn to destination
    		turn_angle(relative_rotation);
    	    // Drive distance in a straight line
    	    drive_distance(distance);
    	    while(1){
    	    	motor_stop();
    	    } // finished
    	}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
