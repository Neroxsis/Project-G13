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
#include <process_image.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


//static uint8_t picked_up = 0;


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
    set_body_led(0);
    set_front_led(0);
    lookup_init();
    process_image_start();


    //start calibration
    //indication that calibration is in progress
    set_led(LED1, 1);
    set_led(LED3, 1);
    set_led(LED5, 1);
    set_led(LED7, 1);
    chThdSleepMilliseconds(1000);

    set_led(LED1, 0);
    calibrate_acc();
    set_led(LED3, 0);
    calibrate_ir();
    set_led(LED5, 0);
    calibrate_gyro();
    set_led(LED7, 0);
    obj_det_init();
    direction_init();


//infinite loop
while(1){


    if(check_order_pointB()){
    	set_led(LED1, 1);
    	chThdSleepMilliseconds(500);
    	set_led(LED3, 1);
    	chThdSleepMilliseconds(500);
    	set_led(LED5, 1);
    	chThdSleepMilliseconds(500);
    	set_led(LED7, 1);


    	turn_angle(get_save_return_angle());
    	// Drive distance in a straight line
    	set_front_led(0);
    	start_search();
    	start_detection();
    	drive_distance(100);

    	end_detection();
    	set_order(0);  // 0 = pointA -> Magic number
    	set_counter_displacement(0);	//do this in direction.c
    	set_x_acc_sign_displacement(0);
    	set_y_acc_sign_displacement(0);
    	set_x_acc_displacement(0);


    	set_led(LED1, 0);
    	chThdSleepMilliseconds(500);
    	set_led(LED3, 0);
    	chThdSleepMilliseconds(500);
    	set_led(LED5, 0);
    	chThdSleepMilliseconds(500);
    	set_led(LED7, 0);
    	chThdSleepMilliseconds(2000);
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
