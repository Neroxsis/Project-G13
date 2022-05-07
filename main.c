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

#include <math.h>


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



uint8_t no_mvmt_detected(float x_speed, float y_speed, float rotation_speed);




extern messagebus_t bus;


static THD_WORKING_AREA(waThdGoalCalculations, 1024); //instead of 128
static THD_FUNCTION(ThdGoalCalculations, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *rotation_topic = messagebus_find_topic_blocking(&bus, "/rotation");
    //int16_t relative_rotation = 0;  //this doesn't work... unknown type name "roatation"

    messagebus_topic_wait(rotation_topic, &relative_rotation, sizeof(relative_rotation));

    systime_t time;
    imu_msg_t imu_values;

    uint16_t no_mvmt_counter = 0;

    float x_speed = 0;  // mm/s
    float y_speed = 0;  // mm/s

    float x_position = 0;
    float y_position = 0;

   // int16_t relative_rotation = 0;
    float  rotation_speed =0;
    int16_t theta = 0; 		// theta = arctan(y/x) to calculate angle_to_goal

    float period = 0.1; // because float * float faster than float * int
    float period_t = 0.1; // for angle calculations; time between two measurements; let's put 1s for now

    int16_t angle_to_goal = 0;
    int32_t distance_to_goal = 0;

    int8_t counter = 0;

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

    //while(in_air){
    while(1){

    	//Prio raufsetzten (To-Do)

    	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));  //updates imu values

    	chprintf((BaseSequentialStream *)&SD3, "____________________________________________ \r\n\n");



    	chprintf((BaseSequentialStream *)&SD3, "Calculate angle \r\n\n");

    	// calculate angle
    	// get_gyro_deg gives you deg/s -> we need to multiply it by a period to get the actual angle in deg
    	chprintf((BaseSequentialStream *)&SD3, "%get_gyro_deg, Z =%d \r\n\n", get_gyro_deg(&imu_values, Z_AXIS));

#define CORRECTION_ROTATION 360/220  // to be calibrated

    	relative_rotation += get_gyro_deg(&imu_values, Z_AXIS) * period_t * CORRECTION_ROTATION;
    	chprintf((BaseSequentialStream *)&SD3, "%relative_rotation, Z =%d \r\n\n", relative_rotation);

    } // temporary to end while loop








 // ----------------------------------------------------------------------------------------------------------------
    	/*


		#define X_ACC_THRESHOLD 0.8  // NOT yet calibrated !!
		#define Y_ACC_THRESHOLD 0.8  // NOT yet calibrated !!
		#define X_SPEED_THRESHOLD 0.08  // NOT yet calibrated !!
		#define Y_SPEED_THRESHOLD 0.08  // NOT yet calibrated !!


    	chprintf((BaseSequentialStream *)&SD3, "Calculate speed \r\n\n");

		chprintf((BaseSequentialStream *)&SD3, " x_acc =%.2f, y_acc =%.2f \r\n\n", imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS]);

		if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD && fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
			x_speed += period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
			y_speed += period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));
		}


		// also calculate rotation speed !!!

		chprintf((BaseSequentialStream *)&SD3, " x_speed  =%.2f \r\n\n", x_speed);
    	chprintf((BaseSequentialStream *)&SD3, " y_speed  =%.2f \r\n\n", y_speed);


    	// PROBLEM: Speed doesn't go back to 0 even if acc. goes back to 0


    	// sets speed to 0 if the robot doesn't move anymore
    		if (fabs(x_speed)<=X_SPEED_THRESHOLD && fabs(y_speed) <= Y_SPEED_THRESHOLD){
    			counter++;
    		}

    		if (counter==10){
    			x_speed=0;
    			y_speed=0;
    			counter=0;
    			chprintf((BaseSequentialStream *)&SD3, " reset speeds to 0 \r\n\n");
    		}
    		chprintf((BaseSequentialStream *)&SD3, " counter %d \r\n\n", counter);



    	chprintf((BaseSequentialStream *)&SD3, "Calculate distance \r\n\n");
    	// calculate distance

    	if (fabs(x_speed) >= X_SPEED_THRESHOLD && fabs(y_speed) >= Y_SPEED_THRESHOLD){
    		x_position += period * x_speed;
    		y_position += period * y_speed;
    	}


    	chprintf((BaseSequentialStream *)&SD3, " x_position=%.2f y_position=%.2f\r\n\n",  x_position, y_position);


    	no_mvmt_detected( x_speed,  y_speed,  rotation_speed);
    	chprintf((BaseSequentialStream *)&SD3, "no mvmt detected %d \r\n\n", no_mvmt_detected( x_speed,  y_speed,  rotation_speed));



    	// test if epuck still in air
    	if(no_mvmt_detected(x_speed, y_speed, rotation_speed)){
    		no_mvmt_counter++;
    		if(no_mvmt_counter > WAIT_TIME){
    			in_air = 0;  // ???
    		}
    	}else{
    		no_mvmt_counter = 0;
    	}


    	 chThdSleepMilliseconds(10);  //maybe we have to change the time ....; in while loop
    }


*/
    // ----------------------------------------------------------------------------------------------------------------





    chThdSleepMilliseconds(10); //prob. remove later

}

uint8_t no_mvmt_detected(float x_speed, float y_speed, float rotation_speed){

#define THRESHHOLD_NO_MVMT 0.05  // still has to be calibrated
#define GYRO_THRESHHOLD 0.05  // still has to be calibrated
	if(x_speed < THRESHHOLD_NO_MVMT &&
		y_speed < THRESHHOLD_NO_MVMT &&
		rotation_speed < THRESHHOLD_NO_MVMT ){
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
    //obj_det_init();


   // messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;


    int16_t relative_rotation = 0;

    // create messgagebus to publish the value of relative_rotation
    messagebus_topic_t rotation_topic;
    MUTEX_DECL(rotation_topic_lock);
    CONDVAR_DECL(rotation_topic_condvar);
    messagebus_topic_init(&rotation_topic, &rotation_topic_lock, &rotation_topic_condvar, &relative_rotation, sizeof(relative_rotation));
    messagebus_advertise_topic(&bus, &rotation_topic, "/rotation");


// change initial_state


    //while(1){
    	chThdSleepMilliseconds(2000);
     //}



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


    chprintf((BaseSequentialStream *)&SD3, "before creation threads \r\n\n");

    //create threads
    chThdCreateStatic(waThdGoalCalculations, sizeof(waThdGoalCalculations), NORMALPRIO, ThdGoalCalculations, NULL);

    //chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO, ThdFrontLed, NULL); should be ok


   //infinite loop
    //while(1){

    	chprintf((BaseSequentialStream *)&SD3, "should start moving soon \r\n\n");

    	uint8_t goal_reached = 0;
    	int16_t relative_rotation = 0;  // in degrees
    	int32_t rotation_steps = 0;
    	float distance = 0; // mm
    	float distance_steps = 0;




    	//while(!in_air(GET) && !goal_reached){


    	    // First turn to destination
    	    right_motor_set_pos(CLEAR);
    	    // Calculate angle in rotation of wheel in steps
    	    rotation_steps =  WHEEL_PERIM * ONE_TURN_STEPS * relative_rotation / (2*PI_DEG);
    	    right_motor_set_speed(TURN_SPEED*sign(rotation_steps));
    	    left_motor_set_speed(-TURN_SPEED*sign(rotation_steps));
    	    do{ // turn right or left       ; I might not need that part ...
    	    	//sleep
    	    }while((abs(rotation_steps) > abs(right_motor_get_pos())));




//    	    // Drive distance in a straight line
//    	    right_motor_set_pos(CLEAR);
//    	    distance_steps = distance * ONE_TURN_STEPS / (WHEEL_PERIM * 10); // *10 as distance is in mm and perim in cm
//    	    right_motor_set_speed(DRIVE_SPEED);
//    	    left_motor_set_speed(DRIVE_SPEED);
//    	    do{
//    	    	//sleep
//    	    	if(get_object_det()){
//    	    		right_motor_set_pos(evade_obj_alg());
//    	    	}
//    	    }while((abs(distance_steps) > abs(right_motor_get_pos())));



    	//}  //from while(!in_air)


    	chThdSleepMilliseconds(2000);


   // } // parenthesis for while loop
}

int32_t evade_obj_alg(void){
/*	int32_t old_pos = right_motor_get_pos();
	do{
		switch(get_object_det()){
			case 0:
				return old_pos; // definitely needs some calculations depending on the size of the object
				break;
			case 1:
				//turn left slowly
				right_motor_set_speed(TURN_SPEED/2);
				left_motor_set_speed(-TURN_SPEED/2);
				while(get_object_det() == 1){
					// sleep
				}
				break;
			case 2:
				//drive forward slowly until sensors lose object
				right_motor_set_speed(DRIVE_SPEED/2);
				left_motor_set_speed(DRIVE_SPEED/2);
				while(get_object_det() == 2){
					// sleep
				}
				break;
			case 3:
				//turn right slowly until object is found again
				right_motor_set_speed(-TURN_SPEED/2);
				left_motor_set_speed(TURN_SPEED/2);
				while(get_object_det() == 3){
					// sleep
				}
				break;
			default:
				panic_handler("object_detection_value");
				return 0;
		}
	}while(get_object_det());
	return old_pos;*/
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
