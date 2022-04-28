#ifndef CONSTANTS_H
#define CONSTANTS_H

// calc gyro
#define RES_250DPS 250
#define MAX_INT16 32768
#define GYRO_RAW2DPS        (RES_250DPS / MAX_INT16)   //250DPS (degrees per second) scale for int16 raw value

// angle
#define PI_DEG 180

// prox
#define FRONT_LEFT_IR_SENSOR 5
#define FRONT_RIGHT_IR_SENSOR 0
#define RIGHT_IR_SENSOR 2
#define RIGHT_BACK_IR_SENSOR 3
#define RIGHT_FRONT_IR_SENSOR 1
#define TH 42

// no magic numbers
#define CLEAR 0
#define OFF 0
#define GET 5

#endif /* CONSTANTS_H */
