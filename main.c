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
#include <direction.h>

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



static imu_msg_t imu_values;

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

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");  //macht man nur einmal


    proximity_start();
    messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");  //macht man nur einmal

    clear_leds();
    set_body_led(0);
    set_front_led(0);
    lookup_init();
    direction_init();


    //start calibration
    //indication that calibration is in progress
    set_led(LED1, 1);
    set_led(LED3, 1);
    set_led(LED5, 1);
    set_led(LED7, 1);


    set_led(LED1, 0);
    calibrate_acc();
    set_led(LED3, 0);
    calibrate_ir();
    set_led(LED5, 0);
    calibrate_gyro();
    set_led(LED7, 0);
    obj_det_init();		// initialize ThdObstacleDetection
    goal_calc_init();	// initialize ThdGoalCalculations


    // are all the Threads initialized ... ?


    proximity_msg_t prox_values;


    float x_speed = 0;  // mm/s
    float y_speed = 0;  // mm/s

    float x_position = 0;
    float y_position = 0;

    static uint8_t goal_reached = 0;
    static int16_t relative_rotation = 0;
    static float distance = 0; // mm



    enum state {pointA, displacement, pointB};
    enum state robot;
    robot = pointA; // robot on start pointA




    //infinite loop
    while(1){

    chprintf((BaseSequentialStream *)&SD3, " in_air = %d \r\n\n", get_in_air());

	set_body_led(1);

// the code from here until where it says "UNTIL HERE " is also in the thread and can be deleted from main
// it's only in main for testing purposes


// --------------------------------------------------------------
    	chprintf((BaseSequentialStream *)&SD3, "____________________________________________ \r\n\n");

    	//updates imu values
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

    	//change so that they are both the same
    	float period_t = 0.1; //10 ms
    	float period = 0.1;

    	// get angle from IMU gyroscope
    	// get_gyro_deg gives you deg/s -> we need to multiply it by a period to get the actual angle in deg
    	chprintf((BaseSequentialStream *)&SD3, "%get_gyro_deg, Z =%d \r\n\n", get_gyro_deg(&imu_values, Z_AXIS));
    	// calculate angle
    	relative_rotation += get_gyro_deg(&imu_values, Z_AXIS) * period_t;
    	chprintf((BaseSequentialStream *)&SD3, "%relative_rotation, Z =%d \r\n\n", relative_rotation);



		#define X_ACC_THRESHOLD 0.08  // NOT yet calibrated !!
		#define Y_ACC_THRESHOLD 0.08  // NOT yet calibrated !!
		#define X_SPEED_THRESHOLD 0.08  // NOT yet calibrated !!
		#define Y_SPEED_THRESHOLD 0.08  // NOT yet calibrated !!


    	chprintf((BaseSequentialStream *)&SD3, "Calculate speed \r\n\n");

		chprintf((BaseSequentialStream *)&SD3, " x_acc =%.2f, y_acc =%.2f \r\n\n", imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS]);

		if (fabs(imu_values.acceleration[X_AXIS]) >= X_ACC_THRESHOLD && fabs(imu_values.acceleration[Y_AXIS]) >= Y_ACC_THRESHOLD){
			x_speed += period * (imu_values.acceleration[X_AXIS] * get_cos(relative_rotation) - imu_values.acceleration[Y_AXIS] * get_sin(relative_rotation));
			y_speed += period * (imu_values.acceleration[X_AXIS] * get_sin(relative_rotation) + imu_values.acceleration[Y_AXIS] * get_cos(relative_rotation));
		}


		chprintf((BaseSequentialStream *)&SD3, " x_speed  =%.2f \r\n\n", x_speed);
    	chprintf((BaseSequentialStream *)&SD3, " y_speed  =%.2f \r\n\n", y_speed);


// --------------------------------------------------------------------------

    	if (!in_air){
    		x_speed=0;
    		y_speed=0;
    		//chprintf((BaseSequentialStream *)&SD3, "set speeds to 0");
    	}

// --------------------------------------------------------------------------

    	// multiply by corrective factor of 100
    	x_position += period * x_speed*100;
    	y_position += period * y_speed*100;

    	chprintf((BaseSequentialStream *)&SD3, " x_position=%.2f y_position=%.2f\r\n\n",  x_position, y_position);


    	distance = sqrt(x_position*x_position + y_position*y_position);
    	chprintf((BaseSequentialStream *)&SD3, " distance=%.2f \r\n\n", distance);


// ------------------------------------------------------------------------

// UNTIL HERE




		//determines state of robot
		if(get_in_air()){
			robot = displacement;
		}
		if(!get_in_air() && robot == pointA){
			robot = pointA;
		}
		if(!get_in_air() && robot == displacement){
			robot = pointB;
		}


// ----------------------------------------------------------------------------

//    	while(!get_in_air() && !goal_reached && robot == pointB){
//       	// First turn towards destination
//    		turn_angle(relative_rotation);
//    	 //Drive distance in a straight line
//    		drive_distance(distance);
//    		goal_reached = 1;
//    	}


    	motor_stop();

    	chThdSleepMilliseconds(10);
    }

}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
