/*
 * calculations.c
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#include <arm_math.h>
#include <constants.h>
#include <calculations.h>

static float sine[91]; // from 0 to 90 degree
static float cosine[91];

// filling out the first quadrant for each degree of sine and cosin
// math.h uses radian: convert degrees to radian
void lookup_init(void){
	for(uint8_t i=0; i<(PI_DEG/2 + 1); i++){
		sine[i] = sin(i*M_PI/PI_DEG); 	// from math.h
		cosine[i] = cos(i*M_PI/PI_DEG);
	}
}


//------------------------------------------------------------------------------------------


// returns for all values a cosine of the value
float get_cos(int16_t angle){
	while(angle >= 360) { angle -= 360; }
	while(angle <= -360) { angle += 360; }
	// angle between without -360 to 360

	if(angle >= 0){
		if(angle <= 90){
			return cosine[angle];
		}else if(angle <= 180){
			return -cosine[PI_DEG - angle];
		}else if(angle <= 270){
			return -cosine[angle - PI_DEG];
		}else if(angle <= 360){ // < 360
			return cosine[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -90){
			return cosine[-angle];
		}else if(angle >= -180){
			return -cosine[PI_DEG + angle];
		}else if(angle >= -270){
			return -cosine[-(PI_DEG + angle)];
		}else if(angle >= -360){ // > -360
			return cosine[2*PI_DEG + angle];
		}
	}
	return 0;
}


//-------------------------------------------------------------------------------------------


// returns for all values a sine of the value
float get_sin(int16_t angle){
	while(angle >= 360) { angle -= 360; }
	while(angle <= -360) { angle += 360; }
	// angle between without -360 to 360
	if(angle >= 0){
		if(angle <= 90){ // Quadrant I
			return sine[angle];
		}else if(angle <= 180){ // Quadrant II
			return sine[PI_DEG - angle];
		}else if(angle <= 270){ // Quadrant III
			return -sine[angle - PI_DEG];
		}else if(angle <= 360){ // < 360  Quadrant IV
			return -sine[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -90){ // Quadrant IV
			return -sine[-angle];
		}else if(angle >= -180){ // Quadrant III
			return -sine[PI_DEG + angle];
		}else if(angle >= -270){ // Quadrant II
			return sine[-(PI_DEG + angle)];
		}else if(angle >= -360){ // > -360  Quadrant I
			return sine[2*PI_DEG + angle];
		}
	}
	return 0;
}


//-----------------------------------------------------------------------------------------


// calculates rotation acceleration from imu_msg_t and returns degree
int16_t get_gyro_deg(imu_msg_t *imu_values, uint8_t axis){
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
	default:
		gyro = 0;	// returns 0 if no real axis is chosen
	}
	return gyro;
}


//-------------------------------------------------------------------------------------------


// returns 1 if the value is positive, -1 if negative and 0 if zero
int8_t sign(int32_t nb){
	if(nb < 0){
		return -1;
	}else if(nb > 0){
		return 1;
	}else{
		return 0;
	}
}


//-------------------------------------------------------------------------------------------


// sign for floating point numbers
int8_t signf(float nb){
	if(nb < 0){
		return -1;
	}else if(nb > 0){
		return 1;
	}else{
		return 0;
	}
}
