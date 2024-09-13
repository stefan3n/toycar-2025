#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"

/*
  Any commands should be added here. When adding a command, always update 'COMMAND_MAX_VALUE' with the last value in the enum.
  Add commands only at the end of the list to avoid changing the order/value of the constants.
*/
struct Response
{
  int data[16];
  int len;
};
// for more infromation about this check Commands.h from SMARTCAEMOTORCONTROL

enum CommandID
{
  CMD_PING = 0, CMD_STOP, CMD_MOVE_ONE, CMD_MOVE_BOTH, CMD_EMERGENCY, CMD_END_EMERGENCY, CMD_ROTATE_RIGHT, CMD_ROTATE_LEFT, 
  CMD_START_POSITION_JETSON, CMD_INITIAL_POS, CMD_CALIBRATE_LEFT, CMD_CALIBRATE_RIGHT, CMD_CLAW_OPEN, CMD_CLAW_CLOSE,
  CMD_ROTATE_CLAW, CMD_FINAL_ARM
};

const int COMMAND_MAX_VALUE = CMD_FINAL_ARM;

enum CommandError
{
  SUCCES_OK,
  ERROR_INVALID_CMD, ERROR_BAD_MSG, ERROR_ARGUMENTS_LENGTH
};

/*
  return - command action error/succes code
  first argument - number of elements in the array passed
  second argument - array of arguments for the function. Can be NULL if there are no arguments.
*/
typedef int(*Command)(int, const int*);
 
/*
  Currently this global variable isn't necesary, any local variable could work.
*/
extern Command scheduledCommand;

Response pingCommand(int len, const int* args);
/*
  Expects no arguments
  Errors: no errors
*/


int stop(int len, const int* args);

/*
  This functions don't need any arguments
  It will stop the motors from moving
  Errors:
    - ERROR_BAD_MSG if the car is in emergency
*/

int moveOne(int len, const int* args);

/*
  inputs:
    2 values: one is the actuator who is moving and another is the new position
  errors:
    ERROR_BAD_MSG if the command is wrong
    ERROR_ARGUMENTS_LENGTH if the command has to few arguments
*/

int moveBoth(int len, const int* args);

/*
  inputs:
    2 values: are the position of the actuators in order:
    first value for first actuator
    second value for second actuator
  errors:
    ERROR_ARGUMENTS_LENGTH if the commnad has insuficient arguments
    ERROR_BAD_MSG if the arm is in emergency or a CMD_FINAL_COMMAND 
                  is running
*/

int emergency_cmd(int len, const int* args);
/*
  inputs:
    - no inputs
  errors:
    - no errors
*/

int end_emergency(int len, const int* args);
/*
  inputs:
    - no inputs
  errors:
    - no errors
*/

int rotateLeft(int len, const int* args);
/*
  inputs: 
      -one positive number witch will be the angle in degree
  errors:
    - ERROR_BAD_MSG if the nuber of steps is negative
      or the arm is in emergency or CMD_FINAL_COMMAND 
      is running
    - ERROR_ARGUMENT_LENGHT if the angle is not send
*/

int rotateRight(int len, const int* args);
/*
  inputs: 
      -one positive number witch will be the angle in degree
  errors:
    - ERROR_BAD_MSG if the nuber of steps is negative
      or the arm is in emergency or CMD_FINAL_COMMAND 
      is running
    - ERROR_ARGUMENT_LENGHT if the angle is not send
*/

int startPositionJetson(int len, const int* args);
/*
  inputs:
    - the startPosition from the jetson
    - the direction
  ERRORS:
    - ERROR_BAD_MSG if the arm is in emergency or CMD_FINAL_COMMAND
    - ERROR_ARGUMENT_LENGHT if the sartPos and direction are not send
*/

int backToInitialPos(int len, const int* args);
/*
  inputs:
    this function don't  need any inputs
  errors:
    - ERROR_BAD_MSG if it is send while emergency state or
      the CMD_FINAL_ARM is executing
*/

int calibrate_left(int len, const int* args);
/*
  inputs:
    -numbers of steps to rotate left
  errors:
    - ERROR_BAD_MSG if the command is sent while emergency state
      or the CMD_FINAL_COMMAND is running
    - ERROR_ARGUMENT_LENGHT if the number of steps is not send
*/

int calibrate_right(int len, const int* args);
/*
  inputs:
    -numbers of steps to rotate left
  errors:
    - ERROR_BAD_MSG if the command is sent while emergency state
      or the CMD_FINAL_COMMAND is running
    - ERROR_ARGUMENT_LENGHT if the number of steps is not send
*/

int openClaw(int len, const int* args);
/*
  inputs:
    - this command don't have any inputs
  errors:
    - ERROR_BAD_MSG if it is send while emergency state or 
      the CMD_FINAL_COMMAND is running
*/

int closeClaw(int len, const int* args);
/*
  inputs:
    -this command don't have any inputs
  errors:
    - ERROR_BAD_MSG if it is send while emergency state or 
      the CMD_FINAL_COMMAND is running
*/

int rotate_claw(int len, const int* args);
/*
  inputs:
    - a value between 0 and 180 that is an angle
  errors:
    - ERROR_ARGUMENTS_LENGTH if the angle is not send
    - ERROR_BAD_MSG if the command is send while emergency state 
      is active or CMD_FINAL_ARM is running
*/

int final_command(int len, const int* args);
/*
  inputs:
    - direction, angle in degree, xPosition, yPosition, angle for claw
  errors:
    - ERROR_BAD_MSG if the command is send while another CMD_FINAL_ARM
      is running or the emergency state is active
    - ERROR_ARGUMENT_LENGHT if the command hasn't all needed
      arguments: direction, angle for stepper, xPosition, yPosition,
      angle for claw
*/


/*
  New command functions should be added here. The value of the CommandID should match the position in this array.
*/

const Command COMMAND_ARRAY[] = {
  pingCommand,          // CMD_PING
  stop,                 // CMD_STOP
  moveOne,              // CMD_MOVE_ONE
  moveBoth,             // CMD_MOVE_BOTH
  emergency_cmd,        // CMD_EMERGENCY
  end_emergency,        // CMD_END_EMERGENCY
  rotateLeft,           // CMD_ROTATE_LEFT
  rotateRight,          // CMD_ROTATE_RIGHT
  startPositionJetson,  // CMD_START_POSITION_JETSON
  backToInitialPos,     // CMD_INITIAL_POS
  calibrate_left,       // CMD_CALIBRATE_LEFT
  calibrate_right,      // CMD_CALIBRATE_RIGHT
  openClaw,             // CMD_CLAW_OPEN
  closeClaw,            // CMD_CLAW_CLOSE
  rotate_claw,          // CMD_ROTATE_CLAW
  final_command         // CMD_FINAL_ARM
};

#endif