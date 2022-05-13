/*
 * process_image.c
 *
 *  Created on: 12.05.2022
 *      Author: Studium
 */


#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <camera/po8030.h>
#include <camera/dcmi_camera.h>
#include <constants.h>
#include <motor.h>
#include <leds.h>

#include <process_image.h>

#define IMAGE_BUFFER_SIZE 640

static uint8_t search = 0;

void has_red(uint8_t* buffer);

static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[2*IMAGE_BUFFER_SIZE] = {0};

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 400, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	while(search){
    		//starts a capture
    		dcmi_capture_start();
    		//waits for the capture to be done
    		wait_image_ready();
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			for(uint16_t i=0; i<(2*IMAGE_BUFFER_SIZE); i+=2){
				//img_buff_ptr points at first address of array of length BUFFER_SIZE*2
				image[i] = (uint8_t) img_buff_ptr[i] & 0xF8; //only red
				image[i+1] = (uint8_t) img_buff_ptr[i+1] & 0x1F; //only blue
			}
			has_red(image);
    	}
    	chThdSleepMilliseconds(50);
    }
}

void has_red(uint8_t* buffer){
	uint32_t mean_red = 0, mean_blue = 0;
	for(uint16_t i = IMAGE_BUFFER_SIZE/4; i<3*IMAGE_BUFFER_SIZE/4; i++){
		mean_red += buffer[2*i];
		mean_blue += buffer[2*i+1];
	}
	mean_red /= (IMAGE_BUFFER_SIZE/2);
	mean_blue /= (IMAGE_BUFFER_SIZE/2);
	if(mean_red > RED_TH && mean_blue < BLUE_TH){
		set_front_led(1);
		set_found_goal(1);
		end_search();
	}
}

/*****************************************Puplic Functions*****************************************/

void process_image_start(void){
	dcmi_start();
	po8030_start();
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
}

void start_search(void){
	search = 1;
}

void end_search(void){
	search = 0;
}

