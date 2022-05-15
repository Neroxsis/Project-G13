/*
 * main.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include "sensors/proximity.h"
#include <leds.h>

#include <constants.h>
#include "calculations.h"
#include <detection.h>
#include "motor.h"
#include <direction.h>
#include <process_image.h>

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
    set_body_led(OFF);
    set_front_led(OFF);
    lookup_init();
    process_image_start();


    //start calibration
    //indication that calibration is in progress
    set_led(LED1, ON);
    set_led(LED3, ON);
    set_led(LED5, ON);
    set_led(LED7, ON);
    chThdSleepMilliseconds(1000);

    set_led(LED1, OFF);
    calibrate_acc();
    set_led(LED3, OFF);
    calibrate_ir();
    set_led(LED5, OFF);
    calibrate_gyro();
    set_led(LED7, OFF);
    obj_det_init();
    direction_init();


	//infinite loop
	while(1){
		// check if robot is at pointB
		if(check_state_pointB()){
			set_leds1357(ON);

			// turn towards starting pointA
			turn_angle(get_angle());

			// put down -> enable object detection and image processing
			start_search();
			start_detection();

			// Drive distance in a straight line
			drive_distance(get_distance());

			// stop object detection
			end_detection();

			// indicates that robot will reset values needed to
			// calculate return_angle (angle robot has to turn to face pointA)
			set_leds1357(OFF);
			set_leds1357(ON);
			set_leds1357(OFF);
			set_front_led(OFF);

			reset_direction();

			// Robot is at new starting pointA
			set_state(pointA);
		}
    chThdSleepMilliseconds(10);
	} // END OF WHILE LOOP
	motors_stop();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
