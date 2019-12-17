#ifndef __RobotSpecs_H_
#define __RobotSpecs_H_

#define WHEEL_RADIUS 0.05
#define WTOW_LENGHT 0.20
#define WTO_WIDTH 0.22

#define ENCODERS_COUNTABLE_EVENTS_MOTOR_SHAFT 28
#define ENCODERS_COUNTABLE_EVENTS_OUTPUT_SHAFT 1993.6
#define MOTOR_OUTPUT_SHAFT_MAX_RPM 84
#define MOTOR_GEAR_RATIO 71.2

#define ODOM_RATE 100 // Hz
#define ODOM_PERIOD 0.01 // s

#define MAX_SPEED 0.44 // m/s
#define TOLERANCE_SPEED 0.3

#define THRESHOLD_DELTA_ENCODERS 100 
#define MAX_DELTA_POSE 0.01

#define MECANUM_Y 1.058361968
#define MECANUM_X 0.9887535903

#endif