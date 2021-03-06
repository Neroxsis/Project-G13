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

// filling out the first quadrant for each degree of sine and cosine
// math.h uses radian: convert degrees to radian
void lookup_init(void){
	for(uint8_t i=0; i<(PI_DEG_F/2 + 1); i++){
		sine[i] = sin(i*M_PI/PI_DEG_F); 	// from math.h
		cosine[i] = cos(i*M_PI/PI_DEG_F);
	}
}


//------------------------------------------------------------------------------------------


// returns for all values a cosine of the value
float get_cos(int16_t angle){
	while(angle >= 2*PI_DEG) { angle -= 2*PI_DEG; }
	while(angle <= -2*PI_DEG) { angle += 2*PI_DEG; }
	// angle between without -360 to 360

	if(angle >= 0){
		if(angle <= PI_DEG/2){
			return cosine[angle];
		}else if(angle <= PI_DEG){
			return -cosine[PI_DEG - angle];
		}else if(angle <= 3*PI_DEG/2){
			return -cosine[angle - PI_DEG];
		}else if(angle <= 2*PI_DEG){ // < 360
			return cosine[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -PI_DEG/2){
			return cosine[-angle];
		}else if(angle >= -PI_DEG){
			return -cosine[PI_DEG + angle];
		}else if(angle >= -3*PI_DEG/2){
			return -cosine[-(PI_DEG + angle)];
		}else if(angle >= -2*PI_DEG){ // > -360
			return cosine[2*PI_DEG + angle];
		}
	}
	return 0;
}


//-------------------------------------------------------------------------------------------


// returns for all values a sine of the value
float get_sin(int16_t angle){
	while(angle >= 2*PI_DEG) { angle -= 2*PI_DEG; }
	while(angle <= -2*PI_DEG) { angle += 2*PI_DEG; }
	// angle between without -360 to 360
	if(angle >= 0){
		if(angle <= PI_DEG/2){ // Quadrant I
			return sine[angle];
		}else if(angle <= PI_DEG){ // Quadrant II
			return sine[PI_DEG - angle];
		}else if(angle <= 3*PI_DEG/2){ // Quadrant III
			return -sine[angle - PI_DEG];
		}else if(angle <= 2*PI_DEG){ // < 360  Quadrant IV
			return -sine[2*PI_DEG - angle];
		}
	}else{	// negative angle
		if(angle >= -PI_DEG/2){ // Quadrant IV
			return -sine[-angle];
		}else if(angle >= -PI_DEG){ // Quadrant III
			return -sine[PI_DEG + angle];
		}else if(angle >= -3*PI_DEG/2){ // Quadrant II
			return sine[-(PI_DEG + angle)];
		}else if(angle >= -2*PI_DEG){ // > -360  Quadrant I
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


//--------------------------------------------------------------------------------------------


// input: x_acc, y_acc measured shortly after robot picked up
//		  angle: relative rotation of robot measured at pointB
//output: degrees that robot has to turn to face the direction of pointA

int16_t return_angle(float x_acc, float y_acc, int16_t angle){
	int16_t theta = CLEAR;

	// - PI <= theta <= PI
	theta = (int16_t)(atan2(y_acc, x_acc)*PI_DEG_F/M_PI);

	// Case 1: Robot is moved along an axis
	// robot moved along y axis -> very small x_acc
	if(fabs(x_acc) <= ACC_THRESHOLD){
		if(y_acc > 0){
			return - angle;		// positive y axis
		}else{
			return PI_DEG_F - angle;	// negative y axis
		}

	// robot moved along x axis -> very small y_acc
	}else if(fabs(y_acc) <= ACC_THRESHOLD){
		if(x_acc > 0){
			return -PI_DEG_F/2 - angle;	//positive x_axis
		}else{
			return PI_DEG_F/2 - angle;	//negative x_axis
		}

	// Case 2: Robot is NOT moved along an axis
	}else{
		//divide into quadrants using sign of x_acc, y_acc
		if(x_acc > 0){
			if(y_acc > 0){
				//quadrant I
				return - PI_DEG_F/2 + theta - angle;
			} else {
				//quadrant IV
				return - PI_DEG_F - theta + angle;
			}
		}

		if(x_acc < 0){
			if(y_acc > 0){
				//quadrant II  OK
				return - PI_DEG_F/2 + theta - angle;
			} else {
				//quadrant III
				return PI_DEG_F/2 + (PI_DEG_F + theta) - angle;
			}
		}
	}

	return 0;

}
