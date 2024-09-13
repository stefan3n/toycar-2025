#include "Commands.h"
#include "HardwareConfiguration.h"
#include <Arduino.h>

Response pingCommand(int len, const int* args)
{
  Response response;

  response.data[1] = SUCCES_OK;
  // set the first argument for message
  // I use abs becuase I cannont send negative numbers
  response.data[2] = abs(startPosition);
  // set the direction left/right

  if(direction == -1)
  {
    response.data[3] = 0;
  }
  else
  {
    response.data[3] = direction;
  }
  // set the lenght of the message
  response.len = 4;
  return response;
}

int stop(int len, const int*  args)
{
  // if I am in emergency state then this command
  // should not have any effect
  if(!emergency)
  {
    // set enable pins for actuators and 
    // stepper to LOW, in this way I stop the
    // movement of the both
    digitalWrite(ENA1_PIN, LOW);
    digitalWrite(ENA2_PIN, LOW);
    digitalWrite(ENA, LOW);
    command = true;
    close = true;
    //init_pos = true;
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int moveBoth(int len, const int* args)
{
  if(command && !emergency)
  {
    //check if I have enough arguments
    if(len < 2)
    {
      return ERROR_ARGUMENTS_LENGTH; 
    }
    digitalWrite(ENA1_PIN, HIGH);
    digitalWrite(ENA2_PIN, HIGH);
    //set the new positions for actuators
    target_position1 = args[0];
    target_position1 =  map(target_position1, 0, 1000, 28, 1023);
    target_position1 = constrain(target_position1, 28, 985);

    target_position2 = args[1];
    target_position2 =  map(target_position2, 0, 1000, 28, 1023);
    target_position2 = constrain(target_position2, 28, 985);
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int moveOne(int len, const int* args)
{
  if(command && !emergency)
  {
    //I check if the codification is right
    if(len < 2)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    //now I set the target position in function of the actuators I have given in command and
    if(args[0] == ACTUATOR1)
    {
      target_position1 = args[1];
      target_position1 =  map(target_position1, 0, 1000, 28, 1023);
      target_position1 = constrain(target_position1, 28, 985);
      digitalWrite(ENA1_PIN, HIGH);
    }
    else if (args[0] == ACTUATOR2)
    {
      target_position2 = args[1];
      target_position2 =  map(target_position2, 0, 1000, 28, 1023);
      target_position2 = constrain(target_position2, 28, 985);
      digitalWrite(ENA2_PIN, HIGH);
    }
    else
    {
      return ERROR_BAD_MSG;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int emergency_cmd(int len, const int* args)
{
  // stet the enable pins for any of them to low
  // and the actuators and stepper stop moving
  digitalWrite(ENA1_PIN, LOW);
  digitalWrite(ENA2_PIN, LOW);
  digitalWrite(ENA, LOW);
  // now I mark that I am in emergency state
  emergency = true;
  return SUCCES_OK;
}

int end_emergency(int len, const int* args)
{
  // I mark that I exit the emergency state
  emergency = false;
  // set enable pins to HIGH and in this way 
  // the actuators and stepper can continue 
  // to move
  digitalWrite(ENA1_PIN, HIGH);
  digitalWrite(ENA2_PIN, HIGH);
  digitalWrite(ENA, HIGH);
  return SUCCES_OK;

}

int rotateLeft(int len, const int* args) 
{
  if(command && !emergency)
  {
    no_calibrate = true;
    //check if the command is right(I have a positive number of steps)
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = (int)((double)args[0] / 0.00375);
      // Now I set the direction and enable pins
      digitalWrite(DIR, LOW);
      digitalWrite(ENA, HIGH);

      //set the target
      if(direction > 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + (stepTarget - stepCount));
      }
      //set the direction
      direction = -1;
      //reset the step count
      stepCount = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int rotateRight(int len, const int* args) 
{
  if(command && !emergency)
  {
    no_calibrate = true;
    //check if the command is right
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = (int)((double)args[0] / 0.00375);
      // set the direction and enable pins
      digitalWrite(ENA, HIGH);
      digitalWrite(DIR, HIGH);

      //set the target
      if(direction < 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + ( stepTarget - stepCount));
      }
      //reset the step counter
      stepCount = 0;
      //set the direction
      direction = 1;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int startPositionJetson(int len, const int* args)
{
  if(command && !emergency)
  {
    if(len < 2)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    // set the position based on the last sign it
    // had when is was last CMD_PING
    if(args[1])
    {
      startPosition = (-1) * args[0];
    }
    else
    {
      startPosition = args[0];
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int backToInitialPos(int len, const int* args)
{
  // check if I am in emergency state or 
  // if the arm is executing a CMD_FINAL_ARM
  // if one of them is true then I return 
  // an error command
  if(command && !emergency)
  {
    // let it to modify the start position
    no_calibrate = true;
    // enable the stepper to move
    digitalWrite(ENA, HIGH);
    // set the target position
    if(startPosition < 0)
    {
      digitalWrite(DIR, HIGH);
      stepTarget = (-1) * startPosition;
      stepCount = 0;
      direction = 1;
    }
    else
    {
        digitalWrite(DIR, LOW);
        stepTarget = startPosition;
        stepCount = 0;
        direction = -1;
    }
    // set enable pins for actuators 
    // to let them to move
    digitalWrite(ENA1_PIN, HIGH);
    digitalWrite(ENA2_PIN, HIGH);
    // set the new targets for actuators
    target_position1 = pos1;
    target_position1 =  map(target_position1, 0, 1000, 28, 1023);
    target_position1 = constrain(target_position1, 28, 985);
    target_position2 = pos2;
    target_position2 =  map(target_position2, 0, 1000, 28, 1023);
    target_position2 = constrain(target_position2, 28, 985);
    // open the claw
    angle.write(180);
    // rotate the claw to it's initial position
    angle_claw.write(0);
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int calibrate_left(int len, const int* args) 
{
  if(command && !emergency)
  {
    //check if the command is right(I have a positive number of steps)
    if(len < 1) 
    {
      return ERROR_BAD_MSG;
    }
    else if(args[0] < 0)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else 
    {
      int steps = args[0];
      // Now I set the direction and enable pins
      digitalWrite(DIR, HIGH);
      digitalWrite(ENA, HIGH);

      //set the target
      if(direction > 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + (stepTarget - stepCount));
      }
      //I set not to count the steps for relative position
      no_calibrate = false;
      //reset the step count
      stepCount = 0;
      //set the direction
      direction = -1;
      //EEPROM.put(0,0);
      startPosition = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int calibrate_right(int len, const int* args) 
{
  if(command && !emergency)
  {
    //check if the command is right
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = args[0];
      // set the direction and enable pins
      digitalWrite(ENA, HIGH);
      digitalWrite(DIR, LOW);

      //set the target
      if(direction < 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + ( stepTarget - stepCount));
      }
      //reset the step counter
      stepCount = 0;
      //I set not to count the steps for relative position
      no_calibrate = false;
      //set the direction
      direction = 1;
      //EEPROM.put(0,0);
      startPosition = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int openClaw(int len, const int* args)
{
  if(command && !emergency)
  {
    //set the maximum angle for claw to open
    angle.write(180);
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int closeClaw(int len, const int* args)
{
  if(command && !emergency)
  {
    // set the minimum angle for claw to close
    angle.write(45);
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int rotate_claw(int len, const int* args)
{
  if(command && !emergency)
  {
    //check if the number of arguments is right
    if(len < 1)
    {
      // the number of arguments is wrong so I send an error message
      return ERROR_ARGUMENTS_LENGTH;
    }
    else
    {
      // now I check if the angle value is right
      if(args[0] < 0 || args[0] > 180)
      {
        // it is not right so I senf an error message
        return ERROR_BAD_MSG;
      }
      else
      {
        // now I rotate the servo
        angle_claw.write(args[0]);
      }
    }
    // I send that the message that the operation is a succes
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int final_command(int len, const int* args)
{
  if(command && !emergency)
  {
    command = false;
    close = true;
    // check if the number of arguments is right
    if(len < 5)
    {
      // I send an error message
      return ERROR_ARGUMENTS_LENGTH;
    }
    else
    {
      int dir = args[0];
      // Now I move the arm before I move the actuators
      // beacuse the actuators are too fast
      if(dir == 0)
      {
        //rotate left
        digitalWrite(DIR, HIGH);
        digitalWrite(ENA, HIGH);
        int steps = (int)((double)args[1] / 0.00375);
        int stepCount = 0;
        while(steps != stepCount)
        {
          digitalWrite(PUL,HIGH); 
          delayMicroseconds(500); 
          digitalWrite(PUL,LOW); 
          delayMicroseconds(500);
          stepCount++;
          startPosition++;
        }
      }
      else
      {
        // rotate right
        digitalWrite(DIR, LOW);
        digitalWrite(ENA, HIGH);
        int steps = (int)((double)args[1] / 0.00375);
        int stepCount = 0;
        while(steps != stepCount)
        {
          digitalWrite(PUL,HIGH); 
          delayMicroseconds(500); 
          digitalWrite(PUL,LOW); 
          delayMicroseconds(500);
          stepCount++;
          startPosition--;
        }
      }
      // set the positions for actuators in function of 
      // x and y of the object
      double x = args[2];// -distance given by links
      double y = args[3];
      // with this series of conditions I can be sure
      // that the arm is not moving too low and hit the
      // claw to the bottle
      if(x-400 >= 11700)
      {
        //Serial.println("here");
        y+=3000;
      }
      else if(x >= 11600)
      {
        y+=2500;
      }
      else if(x >= 11300)
      {
        //Serial.print("here");
        y += 2000;
      }
      else if(x >= 11200)
      {
        //Serial.print("here");
        y += 1500;
      }
      else if(x >= 11000)
      {
        y+= 1200;
      }
      else if(x >= 10200)
      {
        //Serial.println("here");
        y += heigh;
      }
      else if(x >=10100)
      {
        y+=heigh/2;
      }
      //correction for x position
      if(x > 10000)
      {
        x+=800;
      }
      else
      {
        x+=400;
      }
      double r = sqrt(x * x + y * y);
      double t1 = x / sqrt(x * x + y * y);
      double t2 = (l1 * l1 + r * r - l2 * l2) / (2 * l1 * r);
      double t3 = y / sqrt(x * x + y * y);
      double t4 = sqrt(1 - ((l1 * l1 + r * r - l2 * l2) / (2 * l1 * r)) *
                    ((l1 * l1 + r * r - l2 * l2) / (2 * l1 * r)));
      double t5 = (l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2);
      int a1 = (int)(sqrt(d1 * d1 + d2 * d2 - 2 * d1 * d2 * (t3 * t4 - t1 * t2)) - Al1);
      int a2 = (int)(sqrt(d3 * d3 + d4 * d4 - 2 * d3 * d4 * t5) - Al2);
      //set the target positions for actuators
      target_position1 = map(a1, 0, 1000, 28, 983);
      target_position1 = constrain(target_position1, 28, 983);
      target_position2 = map(a2, 0, 1000, 28, 983);
      target_position2 = constrain(target_position2, 28, 983);
      // rotate the claw
      angle_claw.write(args[4]);
      return SUCCES_OK;
    }
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}