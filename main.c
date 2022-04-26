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

#include <arm_math.h>

static float sinus[91]; // from 0 to 90 degree
static float cosinus[91];

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


static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

static THD_WORKING_AREA(waThdGoalCalculations, 128);
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    imu_msg_t imu_values;

    uint16_t no_mvmt_counter = 0;

    float x_speed = 0;  // mm/s oder 10^-1 mm/s
    float y_speed = 0;  // mm/s

    float x_position = 0;
    float y_position = 0;

    int16_t relative_rotation = 0;
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
    	}
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
    	// Read front two Sensors

    	// Test for object

    	// Set object_detected to 1
    }

    while(object_detected){
    	// Switch case 1

        // Read right or left sensor maybe the two further back as well

    	// Test for object

    	// If no sensor detects an object set object_detected to 2

    	// switch case 2

    	// test side sensors while turning

    	// further algorithms
    }
}

static THD_WORKING_AREA(waThdMovement, 128);
static THD_FUNCTION(ThdMovement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint8_t in_air = 0;
    uint8_t goal_reached = 0;

    while(in_air && !goal_reached){
        // First turn to destination

    	// Drive distance in a straight line

    	// Test for obstacle warning
    	// switch to regulate behavior
    }
}

float get_sin(int16_t angle){
#define PI_DEG 180
	static float sinus[91] = {0};
	while(angle >= 360) { angle -= 360; }
	while(angle <= 360) { angle += 360; }
	// angle between without -360 to 360
	if(angle >= 0){
		if(angle <= 90){ // Quadrant I
			return sinus[angle];
		}else if(angle <= 180){ // Quadrant II
			return sinus[PI_DEG - angle];
		}else if(angle <= 270){ // Quadrant III
			return -sinus[angle - PI_DEG];
		}else if(angle <= 360){ // < 360  Quadrant IV
			return -sinus[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -90){ // Quadrant IV
			return -sinus[-angle];
		}else if(angle >= -180){ // Quadrant III
			return -sinus[PI_DEG + angle];
		}else if(angle >= -270){ // Quadrant II
			return sinus[-(PI_DEG + angle)];
		}else if(angle >= -360){ // > -360  Quadrant I
			return sinus[2*PI_DEG + angle];
		}
	}
	return 0;
}

float get_cos(int16_t angle){
#define PI_DEG 180
	static float cosinus[91] = {0};
	while(angle >= 360) { angle -= 360; }
	while(angle <= 360) { angle += 360; }
	// angle between without -360 to 360
	if(angle >= 0){
		if(angle <= 90){
			return cosinus[angle];
		}else if(angle <= 180){
			return -cosinus[PI_DEG - angle];
		}else if(angle <= 270){
			return -cosinus[angle - PI_DEG];
		}else if(angle <= 360){ // < 360
			return cosinus[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -90){
			return cosinus[-angle];
		}else if(angle >= -180){
			return -cosinus[PI_DEG + angle];
		}else if(angle >= -270){
			return -cosinus[-(PI_DEG + angle)];
		}else if(angle >= -360){ // > -360
			return cosins[2*PI_DEG + angle];
		}
	}
	return 0;
}

uint8_t no_mvmt_detected(imu_msg_t* imu_values){
#define THRESHHOLD 42
#define GYRO_THRESHHOLD 2
	if(imu_values->acceleration[X_AXIS] < THRESHHOLD &&
			imu_values->acceleration[Y_AXIS] < THRESHHOLD &&
			imu_values->acceleration[Z_AXIS] < THRESHHOLD &&
			imu_values->gyro_rate[X_AXIS] < GYRO_THRESHHOLD &&
			imu_values->gyro_rate[Y_AXIS] < GYRO_THRESHHOLD &&
			imu_values->gyro_rate[Z_AXIS] < GYRO_THRESHHOLD){
		return 1;
	}else{
		return 0;
	}
}

void initLookup(void){
	for(uint8_t i=0; i<91; i++){
		sinus[i] = sin(i); // from math.h
		cosinus[i] = cos(i);
	}
}

int16_t get_gyro_deg(imu_msg_t *imu_values, uint8_t axis){
#define RES_250DPS 250
#define MAX_INT16 32768
#define GYRO_RAW2DPS        (RES_250DPS / MAX_INT16)   //250DPS (degrees per second) scale for int16 raw value
	int16_t gyro = 0;
	switch(axis){
	case X_AXIS:
		gyro = (imu_values->gyro_raw[X_AXIS] - imu_values->gyro_offset[X_AXIS]) * GYRO_RAW2DPS;
		break;
	case Y_AXIS:
		gyro = (imu_values->gyro_raw[Y_AXIS] - imu_values->gyro_offset[Y_AXIS]) * GYRO_RAW2DPS;
		break;
	case Z_AXIS:
		gyro = (imu_values->gyro_raw[Z_AXIS] - imu_values->gyro_offset[Z_AXIS]) * GYRO_RAW2DPS;
		break;
	}
	return gyro;
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //starts timer 12
    timer12_start();
    //inits the motors
    motors_init();

    chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);
    //messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
