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
#define IR_THRESHHOLD 100
#define FALSE_ALARM 1000

// no magic numbers
#define CLEAR 0
#define OFF 0

// imu
#define INCLINE_TH -0.5f
#define DIVIDE_BYZ 0.1f

// camera
#define RED_TH 100
#define BLUE_TH 10

// directions
#define GRAVITY 9.81
//should I always put an f at the end?
#define X_ACC_THRESHOLD 0.07
#define Y_ACC_THRESHOLD 0.07
#define X_ROTATION_THRESHOLD 1.5
#define Y_ROTATION_THRESHOLD 1.5
#define ROTATION_THRESHOLD 0.001f
#define SPEED_MMPCS 3 // 300 mm per s or 3 mm per cs (10^-2 s)

// main
#define Z_ACC_THRESHOLD 2 //not yet calibrated maybe even lower

#endif /* CONSTANTS_H */
