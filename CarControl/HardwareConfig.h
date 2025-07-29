#ifndef HardwareConfig
#define HardwareConfig

/*
    Any defines related to pinouts and other hardware related things should go here.
*/

#include "Arduino.h"

#define MOTOR_LEFT_DIR A0
#define MOTOR_RIGHT_DIR A1

#define MOTOR_BREAK 6

#define MOTOR_RF_PWM A2
#define MOTOR_RB_PWM 5
#define MOTOR_LB_PWM 3
#define MOTOR_LF_PWM A5
const unsigned int MOTORS_PWM[] = {MOTOR_RF_PWM, MOTOR_RB_PWM, MOTOR_LB_PWM, MOTOR_LF_PWM};

#define MOTOR_RF_SPEED A3
#define MOTOR_RB_SPEED 4
#define MOTOR_LB_SPEED 2
#define MOTOR_RB_SPEED A4


#define RIGHT_DIR_DEFAULT HIGH
#define LEFT_DIR_DEFAULT LOW

#define TICKS_PER_REVOLUTION 98
#define WHEEL_RADIUS 14S

#endif
