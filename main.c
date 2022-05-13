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
    //direction_init();


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


    //messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    //proximity_msg_t prox_values;



    enum state {pointA, displacement, pointB};
    enum state order;
    order = pointB;

    static uint8_t goal_reached = 0;



    //infinite loop
    while(1){ 			//while(order != pointB){

//    	chprintf((BaseSequentialStream *)&SD3, "order = %d \r\n\n", order);
//    	chprintf((BaseSequentialStream *)&SD3, "in_air = %d \r\n\n", in_air());
//
//
//    	if (in_air()){
//    		order = displacement;
//    	} else {
//    		if (order == displacement){
//    			order = pointB;
//    		}
//    	}



//    	chprintf((BaseSequentialStream *)&SD3, "relative_rotation = %.2f \r\n\n", get_relative_rotation());
//    	chprintf((BaseSequentialStream *)&SD3, "distance = %.2f \r\n\n", get_distance());


    	if(order == pointB && !goal_reached){
    		// First turn to destination
    	    //turn_angle(90);
    	   	// Drive distance in a straight line
    		start_search();
    	   	drive_distance(300);
    		//goal_reached = 1;
    		//set_led(LED1, 1);
        	chThdSleepMilliseconds(5000);

    	}
    	//set_led(LED3, 1);
    }
    //motor_stop();

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
