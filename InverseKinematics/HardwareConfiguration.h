#include <Servo.h>

//define pins

// for actuator

// input signal first acutaor
#define IN1_PIN 6
#define IN2_PIN 5
// enable pin for first actuator
#define ENA1_PIN 7 
// enable pin for second actuator
#define ENA2_PIN 8 
// input signal second actuator
#define IN3_PIN 10
#define IN4_PIN 9

//these ones are for the feedback
#define POTENTIOMETER_PIN1 A0 // feedback pin for first actuator
#define POTENTIOMETER_PIN2 A1 // feedback pin for second actuator

//these are for stepper
#define DIR 2 // direction
#define PUL 3 // pulse
#define ENA 4 // enable

//this pin is for claw
#define ROTATE_CLAW 11 // for MG90S
#define CLAW 13 // for MG966R

//I define error_margin
#define ERROR_MARGIN 5

//I define a code for every actuator as name
#define ACTUATOR1 1
#define ACTUATOR2 2

//I set a delay time to rotate the arm
#define stepDelay 500

// number of seconds while the arm will not response
// for some command after recieving CMD_FINAL_ARM
#define BlockingTime 5

// this variable are constants used for forward kinematics 
// and they are mesured 
extern double d1;
// the distance from intersection point between first actuator and bottom horizontal link and first join
extern double d2;
// the distance form intersection point between first actuator and vertical link and first join
extern double d3;
// the distance from intersection point between second actuator and vertical link and second join
extern double d4; 
// the distance from intersection point between second actuator and upper horizontal link and second join  
extern double Al1; // the lenght of the first actuator
extern double Al2; // the lenght of the second actuator

extern double l1; // the length of the first link
extern double l2; // the lenght of the second link

// This variables will keep the new position
extern int target_position1;
extern int target_position2;


//this variables will keep the current position for every actuator
extern int feedback_value1;
extern int feedback_value2;

extern int stepTarget; // this one will give the position
extern int stepCount; // this will count the numbers of steps from the moment I send the command
extern int startPosition; // this is the variabl for initial position
extern int direction; // this variable will be used to compare the number of steps relative at the initial position
extern int pos1;//this will keep the initial positon for 1st actuaor
extern int pos2;//this will keept the initial position for 2nd actuator
extern Servo angle; // this variable is used to command the claw
extern Servo angle_claw; // this variable is used to rotate the claw
extern bool no_calibrate;// this one will tell me if I calibrate the arm manualy or not

extern double heigh; // this variable is used to prevent the claw to go too down and hit the ground
extern bool command; //  this variable is used to keep last command and in this way I make finalCommand blocking excpet stop command]
extern long time; // this will keep a time while the arm will not response
extern int time_claw; // this will close the claw after a specific time
extern bool close; // this will close the claw after a period of timea
extern bool emergency; // this will tell me if I am in emergncy state or not