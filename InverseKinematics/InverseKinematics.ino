#include "HardwareConfiguration.h"
#include "Commands.h"
#include "Communication.h"

// Global variables from HardwareConfiguration.h
int feedback_value1 = 0;
int feedback_value2 = 0;
int feedback_value3 = 0;

int ret1 = SUCCES_OK;
int ret2 = SUCCES_OK;
int ret3 = SUCCES_OK;

int target_position1 = 0;
int target_position2 = 0;
int target_position3 = 0;

int stepCount = 0;
int direction = 1;
int stepTarget = 0;

int startPosition = 0;
Servo angle;
Servo angle_claw;

const int currentPosition = 0;

double l1 = 49.5 * 100;
double l2 = 49.5 * 100;
double l3 = 49.5 * 100;

//this variable is a treshold for the actuator
double heigh = 1000;

//this will keep the y position of the claw
double y = 0;

bool no_calibrate = true;
bool command = true;
long time = 0;
int time_claw = 0;
bool close = false;
bool emergency = false;

int pos1 = 0;
int pos2 = 1000;

Command scheduledCommand = NULL;


void setup() 
{
  Serial.begin(9600);
 

  // Actuators
  pinMode(IN1_A1, OUTPUT);
  pinMode(IN2_A1, OUTPUT);
  pinMode(ENA_A1, OUTPUT);
  
  pinMode(IN1_A2, OUTPUT);
  pinMode(IN2_A2, OUTPUT);
  pinMode(ENA_A2, OUTPUT);

  pinMode(IN1_A3, OUTPUT);
  pinMode(IN2_A3, OUTPUT);
  pinMode(ENA_A3, OUTPUT);

  // Open claw
  angle.attach(CLAW); 
  
  /* ================ ISN'T USED ANYMORE ================
  angle_claw.attach(ROTATE_CLAW); // rotate to 0 degree
     ==================================================== */

  // Stepper
  pinMode(DIRECTION_STEPPER, OUTPUT);
  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(ENA_STEPPER, OUTPUT);

  // Default values for positions
  feedback_value1 = analogRead(FEEDBACK_A1);
  feedback_value2 = analogRead(FEEDBACK_A2);
  feedback_value3 = analogRead(FEEDBACK_A3);

  target_position1 = feedback_value1;
  target_position2 = feedback_value2;
  target_position3 = feedback_value3;

  backToInitialPos(msgLen, msg);

  // Default claw position
  angle.write(180);

  // Timeout
  time = millis()/1000;
}

void loop() 
{
  if(!emergency)
  {
    if(ret1 == SUCCES_OK)
    {
      // Check if position is ok for (1) first actuator
      if((abs(feedback_value1 - target_position1)) > ERROR_MARGIN)
      {
        // Move if not
        moveActuator(ACTUATOR1, target_position1);
      }
      else
      {
        // Stop the actuator
        digitalWrite(IN1_A1, LOW);
        digitalWrite(IN2_A1, LOW);
      }

      
      // Check if position is ok for (2) second actuator
      if((abs(feedback_value2 - target_position2)) > ERROR_MARGIN)
      {
        // Move if not
        moveActuator(ACTUATOR2, target_position2);
      }
      else
      {
        // Stop the actuator
        digitalWrite(IN1_A2, LOW);
        digitalWrite(IN2_A2, LOW);
      }

      // Check if position is ok for (3) third actuator
      if((abs(feedback_value3 - target_position3)) > ERROR_MARGIN)
      {
        // Move if not
        moveActuator(ACTUATOR3, target_position3);
      }
      else
      {
        // Stop the actuator
        digitalWrite(IN1_A3, LOW);
        digitalWrite(IN2_A3, LOW);
      }
    }

    if(ret2 == SUCCES_OK)
    {
      // Continue rotation if steps are not enough
      if(stepCount != stepTarget && digitalRead(ENA_STEPPER) == HIGH)
      {
        // Move one step
        digitalWrite(PULSE_STEPPER,HIGH); 
        delayMicroseconds(500); 

        digitalWrite(PULSE_STEPPER,LOW); 
        delayMicroseconds(500);
        
        stepCount++;
        if(no_calibrate)
        {
          startPosition += direction;
        }
      }
      else
      {
        // Disable stepper
        digitalWrite(ENA_STEPPER, LOW);
      }

      // Finish the CMD_FINAL_ARM command
      if(millis()/1000 - time_claw >= 4 && !command && close)
      {
        // Close the claw
        angle.write(45);
        
        // Mark that the claw is closed
        close = false;

        // Delay such that the claw has enough time to close
        delay(1000);

        // Rise the object
        target_position1 = 28;
      }
    }
  }

  int msgRecieve = receiveByte();

  // Check if msg received
  if(msgRecieve)
  {
    if(msg[1] == CMD_PING)
    {
        Response response = pingCommand(msgLen-2, msg+2);
        writeMessage(response.len, response.data);
    }
    else if(msg[1] <= CMD_MOVE_ALL) 
    {
      scheduledCommand = COMMAND_ARRAY[msg[1]];
      if(scheduledCommand != NULL)
      {
        ret1  = scheduledCommand(msgLen - 2, msg + 2);
      }
      int response[] = {0, ret1};
      writeMessage(2, response);
    }
    else if(msg[1] < CMD_FINAL_ARM)
    {
      Serial.print(msg[1]);
      scheduledCommand = COMMAND_ARRAY[msg[1]];
      if(scheduledCommand != NULL)
      {
        ret2  = scheduledCommand(msgLen - 2, msg + 2);
        int response[] = {0, ret2};
        writeMessage(2, response);
      }
    }
    else if(msg[1] == CMD_FINAL_ARM && command == true && !emergency)
    {
      if(millis()/1000 - time >= BlockingTime || time == 0)
      {
        angle.write(180);
        time = millis()/1000;
        time_claw = millis()/1000;
        ret2 = COMMAND_ARRAY[msg[1]](msgLen - 2, msg + 2);
        int response[] = {0, ret2};
        writeMessage(2, response);
      }
    }
    else
    {
      ret2 = ERROR_BAD_MSG;
      int response[] = {0, ret2};
      writeMessage(2, response);
    }
  }

  // Timeout for CMD_FINAL
  if(millis()/1000 - time >=  BlockingTime || emergency)
  {
    command = true;
  }
}


// Actuator movement
void moveActuator(int actuator, int position)
{
  int pin1;
  int pin2;
  int potentimeter_pin;
  int feedback_value;

  //set the values from above
  if(actuator == ACTUATOR1)
  {
    feedback_value = feedback_value1 = analogRead(FEEDBACK_A1);
    pin1 = IN1_A1;
    pin2 = IN2_A1;
    potentimeter_pin = FEEDBACK_A1;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value = feedback_value2 = analogRead(FEEDBACK_A2);
    pin1 = IN1_A2;
    pin2 = IN2_A2;
    potentimeter_pin = FEEDBACK_A2;
  }
  else if(actuator == ACTUATOR3) 
  {
    feedback_value = feedback_value3 = analogRead(FEEDBACK_A3);
    pin1 = IN1_A3;
    pin2 = IN2_A3;
    potentimeter_pin = FEEDBACK_A3;
  }

  // Establish the move    
  if(feedback_value < position)
  {
    // Extend
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else
  {
    // Retract
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  feedback_value = analogRead(potentimeter_pin);

  // Set the new position for feedback value for the moved actuator
  if(actuator == ACTUATOR1)
  {
    feedback_value1 = feedback_value;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value2 = feedback_value;
  }
  else if(actuator == ACTUATOR3)
  {
    feedback_value3 = feedback_value;
  }
}

/* =================================== ISN'T USED ANYMORE ===================================
// Check using forward kinematics if is to close to the ground
bool check_y(int position1, int position2)
{
  // Calculate the length of actuators
  double a1 = Al1 + map(position1, 28, 985, 0, 1000);
  double a2 = Al2 + map(position2, 28, 985, 0, 1000);
  // this terms are obtained by using math and will help me to calculate the y position
  double r1 = sqrt(1 - ((d1 * d1 + d2 * d2 - a1 * a1) / (2 * d1 * d2)));
  double r2 = sqrt(1 - ((d3 * d3 + d4 * d4 - a2 * a2) / (2 * d3 * d4)));
  double t1 = (d1 * d1 + d2 * d2 - a1 * a1) / (2 * d1 * d2);
  double t2 = (d3 * d3 + d4 * d4 - a2 * a2) / (2 * d3 * d4);
  // now I calculate the y by founded formula
  y = l1 * r1 + l2 * (t1 * r2 - r1 * t2);
  // now I send the message to move the arm or not
  if(y < heigh)
  {
    return false;
  }
  else
  {
    return true;
  }
}
 ============================================================================================ */