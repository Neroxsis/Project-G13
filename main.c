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

#include <arm_math.h>

#include <constants.h>
#include "calculations.h"

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

static THD_WORKING_AREA(waThdObstacleDetection, 128);
static THD_FUNCTION(ThdObstacleDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t object_detected = 0;

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
    			// Read right or left sensor maybe the two further back as well
    			// If no sensor detects an object set object_detected to 2
    			if(get_calibrated_prox(RIGHT_IR_SENSOR) < TH && get_calibrated_prox(RIGHT_BACK_IR_SENSOR) > TH){
    				object_detected = 2;
    			}
    			break;
    		case 2:
    			if(get_calibrated_prox(RIGHT_IR_SENSOR) > TH){
    				object_detected = 1;
    			}
    			break;	// test side sensors while turning
    						// further algorithms to set object_detected to 0
    		default:
    			break;
    	}
    }
}

uint8_t no_mvmt_detected(imu_msg_t* imu_values){
#define THRESHHOLD 42
#define GYRO_THRESHHOLD 2
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
    init_lookup();

    //start calibration
    //indication that calibration is in progress
    set_rgb_led(LED2,RGB_MAX_INTENSITY,0,0);
    set_rgb_led(LED4,RGB_MAX_INTENSITY,0,0);
    set_rgb_led(LED6,RGB_MAX_INTENSITY,0,0);
    set_rgb_led(LED8,RGB_MAX_INTENSITY,0,0);

    calibrate_gyro();
    set_rgb_led(LED2,0,RGB_MAX_INTENSITY,0);
    calibrate_acc();
    set_rgb_led(LED8,0,RGB_MAX_INTENSITY,0);
    calibrate_ir();
    set_rgb_led(LED4,0,RGB_MAX_INTENSITY,0);
    set_rgb_led(LED6,0,RGB_MAX_INTENSITY,0);

    //sleep 1 sec
    clear_leds();

    //create threads
    chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
    chThdCreateStatic(waThdObstacleDetection, sizeof(waThdObstacleDetection), NORMALPRIO, ThdObstacleDetection, NULL);
    //messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    //infinite loop
    while(1){
    	uint8_t goal_reached = 0;
    	int16_t relative_rotation = 0;
    	int32_t rotation_steps = 0;
    	float distance = 0; // mm
    	float distance_steps = 0;

    	while(!in_air && !goal_reached){
    	    // First turn to destination
    	    right_motor_set_pos(0);
    	    // Calculate angle in rotation of wheel in steps
    	    rotation_steps =  WHEEL_PERIM * ONE_TURN_STEPS * relative_rotation / (2*PI_DEG);
    	    right_motor_set_speed(TURN_SPEED*sign(rotation_steps));
    	    left_motor_set_speed(-TURN_SPEED*sign(rotation_steps));
    	    do{ // turn right or left
    	    	//sleep
    	    }while((abs(rotation_steps) > abs(right_motor_get_pos())));

    	    // Drive distance in a straight line
    	    right_motor_set_pos(0);
    	    distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm
    	    right_motor_set_speed(DRIVE_SPEED);
    	    left_motor_set_speed(DRIVE_SPEED);
    	    do{
    	    	//sleep
    	    	if(object_detection){
    	    		right_motor_set_pos(evade_obj_alg());
    	    	}
    	    }while((abs(distance_steps) > abs(right_motor_get_pos())));
    	    // Test for obstacle warning
    	    // switch to regular behavior
    	}
    }
}

int32_t evade_obj_alg(void){
	int32_t old_pos = right_motor_get_pos();
	do{
		switch(object_detection){
			case 0:
				return old_pos; // definitely needs some calculations depending on the size of the object
				break;
			case 1:
				//turn left slowly
				right_motor_set_speed(TURN_SPEED/2);
				left_motor_set_speed(-TURN_SPEED/2);
				while(object_detection == 1){
					// sleep
				}
				break;
			case 2:
				//drive forward slowly until sensors lose object
				right_motor_set_speed(DRIVE_SPEED/2);
				left_motor_set_speed(DRIVE_SPED/2);
				while(object_detection == 2){
					// sleep
				}
				break;
			case 3:
				//turn right slowly until object is refound
				right_motor_set_speed(-TURN_SPEED/2);
				left_motor_set_speed(TURN_SPEED/2);
				while(object_detection == 3){
					// sleep
				}
				break;
			default:
				panik_handler(&"object_detection_value");
				return 0;
		}
	}while(object_detection);
	return old_pos;
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
