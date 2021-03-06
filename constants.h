/*
 * constants.h
 *
 * Author: Dominik Helbing, Simona Herren
 * Group: G13
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

// calc gyro
#define RES_250DPS 250.0f
#define MAX_INT16 32768.0f
#define GYRO_RAW2DPS        (RES_250DPS / MAX_INT16)   //250DPS (degrees per second) scale for int16 raw value

// angle
#define PI_DEG 180
#define PI_DEG_F 180.0f
#define FULL_TURN 360
#define HALF_TURN 180
#define QUARTER_TURN 90

// prox
#define FRONT_RIGHT_IR_SENSOR 0
#define RIGHT_FRONT_IR_SENSOR 1
#define RIGHT_IR_SENSOR 2
#define RIGHT_BACK_IR_SENSOR 3
#define FRONT_LEFT_IR_SENSOR 7
#define IR_THRESHOLD 100
#define FALSE_ALARM 1000

// no magic numbers
#define CLEAR 0
#define OFF 0
#define ON 1

// object detected
#define EVADE_TURN_LEFT 1
#define EVADE_DRIVE_STRAIGHT 2
#define EVADE_TURN_RIGHT 3
#define EVADE_FA 4

// camera
#define IMAGE_BUFFER_SIZE 640
#define RED_TH 110
#define BLUE_TH 10

// directions
#define GRAVITY 9.81
#define X_ACC_THRESHOLD 0.07f
#define Y_ACC_THRESHOLD 0.07f
#define Z_ACC_THRESHOLD 2.0f
#define ACC_THRESHOLD 0.08f
#define X_ROTATION_THRESHOLD 1.5f
#define Y_ROTATION_THRESHOLD 1.5f
#define ROTATION_THRESHOLD 0.001f
#define SPEED_MMPCS 3 // 300 mm per s or 3 mm per cs (10^-2 s)
#define DISTANCE_THRESHOLD 20.0f //mm
#define PERIOD 0.012f //s
#define IN_AIR 1
#define ON_GROUND 0
#define MAX_COUNTER_DISPLACEMENT 30
#define COMPENSATION 360.0f/210.0f // one turn gives a rotation of 210 degree instead of 360

//motor
#define ONE_TURN_STEPS 1300
#define WHEEL_PERIM 13 //cm
#define TURN_SPEED 800
#define DRIVE_SPEED 900
#define EVADE_DISTANCE 4 //cm
#define DIST_TO_GOAL 15 //cm
#define FAST 1
#define SLOW 2

#endif /* CONSTANTS_H */
