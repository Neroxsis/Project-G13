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


static uint8_t picked_up = 0;


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
    direction_init();


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
    //thred init. GoalCalculatins



    //messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    //proximity_msg_t prox_values;



    enum state {pointA, displacement, pointB};
    enum state order;
    order = pointA;

    int8_t counter_deceleration = 0;
    int8_t counter_small_acc = 0;

//infinite loop
while(1){


// ----------------------------------------------------------------------
//	Determines if robot is picked up and then turns on body lights

	if (fabs(get_z_axis_acc() + GRAVITY) < Z_ACC_THRESHOLD){
		counter_small_acc++;
		if(counter_small_acc == 10){
			picked_up = 0;
			set_body_led(picked_up);
		}
	}

	if ((get_z_axis_acc() + GRAVITY) <= -Z_ACC_THRESHOLD){
		if(picked_up == 0){
			picked_up = 1;
			set_body_led(picked_up);
		}else{

			counter_deceleration++;
			if (counter_deceleration==8){
				picked_up = 0;
				set_body_led(picked_up);
			}
		}
	}
	set_body_led(picked_up);
// ----------------------------------------------------------------------


    while(order != pointB){		//while(1)
    	if (picked_up){
    		order = displacement;
    	} else {
    		if (order == displacement){
    			order = pointB;
    		}
    	}
    }

    chprintf((BaseSequentialStream *)&SD3, "__________________________________________ \r\n\n");
    chprintf((BaseSequentialStream *)&SD3, "order = %d \r\n\n", order);
    chprintf((BaseSequentialStream *)&SD3, "picked up = %d \r\n\n", picked_up);
    chprintf((BaseSequentialStream *)&SD3, "relative_rotation = %.2f \r\n\n", get_relative_rotation());
    	chprintf((BaseSequentialStream *)&SD3, "distance = %.2f \r\n\n", get_distance());
    	chprintf((BaseSequentialStream *)&SD3, "x_speed = %.2f \r\n\n", get_x_speed());
    	chprintf((BaseSequentialStream *)&SD3, "y_speed = %.2f \r\n\n", get_y_speed());
    	chprintf((BaseSequentialStream *)&SD3, "x_position = %.2f \r\n\n", get_x_position());
    	chprintf((BaseSequentialStream *)&SD3, "y_position = %.2f \r\n\n", get_y_position());
//
//
//
//    if(order == pointB){
//    	 set_led(LED1, 1);
//    	 chThdSleepMilliseconds(500);
//    	 set_led(LED3, 1);
//    	 chThdSleepMilliseconds(500);
//    	 set_led(LED5, 1);
//    	 chThdSleepMilliseconds(500);
//    	 set_led(LED7, 1);
//    	 chThdSleepMilliseconds(4000);
//
//    	//First turn to destination
//    	chprintf((BaseSequentialStream *)&SD3, "relative_rotation = %.2f \r\n\n", get_relative_rotation());
//    	turn_angle(-get_relative_rotation()); //get_relative_rotation()
//    	// Drive distance in a straight line
//    	drive_distance(100);
//    	motor_stop();		// THIS WILL BE INCLUDED IN THE NEW MOTOR.C FILE FROM DOMINIK
//    	order = pointA;
//
//    	set_led(LED1, 0);
//    	set_led(LED3, 0);
//    	set_led(LED5, 0);
//    	set_led(LED7, 0);
//    	chThdSleepMilliseconds(5000);
//
//    	chprintf((BaseSequentialStream *)&SD3, "__________________________________________ \r\n\n");
//    	chprintf((BaseSequentialStream *)&SD3, "order = %d \r\n\n", order);
//
//
//    }


		//chThdSleepMilliseconds(10);


    chThdSleepMilliseconds(10);
	} // END OF WHILE LOOP

motor_stop();

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
