#ifndef CONSTANTS_H
#define CONSTANTS_H

// calc gyro
#define RES_250DPS 250.0f
#define MAX_INT16 32768.0f
#define GYRO_RAW2DPS        (RES_250DPS / MAX_INT16)   //250DPS (degrees per second) scale for int16 raw value

// angle
#define PI_DEG 180

// prox
#define FRONT_RIGHT_IR_SENSOR 0
#define RIGHT_FRONT_IR_SENSOR 1
#define RIGHT_IR_SENSOR 2
#define RIGHT_BACK_IR_SENSOR 3
#define FRONT_LEFT_IR_SENSOR 7
#define IR_THRESHHOLD 20
#define FALSE_ALARM 1000

// no magic numbers
#define CLEAR 0
#define OFF 0

// imu
#define INCLINE_TH -0.5f
#define DIVIDE_BYZ 0.1f


//// mvmt
////#define ROTATION_TH 2
//#define ONE_TURN_STEPS 100
//#define WHEEL_PERIM 13 //cm
//#define TURN_SPEED 800
//#define DRIVE_SPEED 1000

#endif /* CONSTANTS_H */
