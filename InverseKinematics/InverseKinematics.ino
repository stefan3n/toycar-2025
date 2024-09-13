#include "HardwareConfiguration.h"
#include "Commands.h"
#include "Communication.h"

// define global variables from HardwareConfiguration
int feedback_value1 = 0;
int feedback_value2 = 0;

int ret1 = SUCCES_OK;
int ret2 = SUCCES_OK;

int target_position1 = 0;
int target_position2 = 0;

int stepCount = 0;
int direction = 1;
int stepTarget = 0;

int startPosition = 0;
Servo angle;
Servo angle_claw;

const int currentPosition = 0;

double d1 = 27 * 100;
double d2 = 23.5 * 100;
double d3 = 28 * 100;
double d4 = 38 * 100;
double Al1 = 36 * 100;
double Al2 = 31.1 * 100;
double l1 = 65 * 100;
double l2 = 102 * 100;

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
  // put your setup code here, to run once:
  Serial.begin(9600);
  // now I set the pin mode for every pin

  //actuators
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // enable pins for actuators
  pinMode(ENA1_PIN, OUTPUT);
  pinMode(ENA2_PIN, OUTPUT);

  //servos
  angle.attach(CLAW); // open claw
  angle_claw.attach(ROTATE_CLAW); // rotate to 0 degree

  //stepper
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(ENA, OUTPUT);

  //EEPROM.put(0,0);
  //set the default values for positions
  feedback_value1 = analogRead(POTENTIOMETER_PIN1);
  feedback_value2 = analogRead(POTENTIOMETER_PIN2);
  target_position1 = feedback_value1;
  target_position2 = feedback_value2;
  //startPosition = EEPROM.get(0, startPosition);
  backToInitialPos(msgLen, msg);

  //default position
  angle.write(180);
  angle_claw.write(0);

  // this is used for timeout
  time = millis()/1000;
}

void loop() 
{
  if(!emergency)
  {
    //Serial.println(startPosition);
    if(ret1 == SUCCES_OK)
    {
      //I check if I am at the position I want
      if((abs(feedback_value1 - target_position1)) > ERROR_MARGIN)
      {

        // if it isn't there I countiune the movment
        moveActuator(ACTUATOR1, target_position1);
      }
      else
      {
        // I stop the acutator becuase I am at the position I want
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
      }


      //I check if I am at the position I want
      if((abs(feedback_value2 - target_position2)) > ERROR_MARGIN)
      {
        // if it isn't there I countiune the movment
        moveActuator(ACTUATOR2, target_position2);
      }
      else
      {
        // I stop the acutator becuase I am at the position I want
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
      }
    }

    if(ret2 == SUCCES_OK)
    {
      //If I don't make the number of steps
      //then I shoul contiune the rotation
      if(stepCount != stepTarget && digitalRead(ENA) == HIGH)
      {
        //this will move with one step
        digitalWrite(PUL,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(PUL,LOW); 
        delayMicroseconds(500);
        //increrase the number of steps
        stepCount++;
        if(no_calibrate)
        {
          startPosition += direction;
          //EEPROM.put(0, startPosition);
        }
      }
      else
      {
        // this command will set the enable low
        // if the stepper is not moving
        digitalWrite(ENA, LOW);
      }

      // now I finnish the CMD_FINAL_ARM command
      if(millis()/1000 - time_claw >= 4 && !command && close)
      {
        // close the claw
        angle.write(45);
        // mark that the claw is closed
        close = false;
        // introduce a delay such that the claw
        // has enough time to close
        delay(1000);
        // rise the bottle
        target_position1 = 28;
      }
    }
  }

  int msgRecieve = receiveByte();

  //check if I recieve something
  if(msgRecieve)
  {
    if(msg[1] == CMD_PING)
    {
        Response response = pingCommand(msgLen-2, msg+2);
        writeMessage(response.len, response.data);
    }
    //check if the given command is for actuators
    else if(msg[1] <= CMD_MOVE_BOTH) 
    {
      //the command is right then, I set the command and execute it
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

  // timeout for CMD_FINAL
  if(millis()/1000 - time >=  BlockingTime || emergency)
  {
    command = true;
  }
}


//this function will move the actuator
void moveActuator(int actuator, int position)
{
  //I set the pins and the feedback value in function of the actuator given
  // input pins
  int pin1;
  int pin2;
  // feedback pins
  int potentimeter_pin;
  // the current position for acutator who is moving
  int feedback_value;

  //set the values from above
  if(actuator == ACTUATOR1)
  {
    feedback_value = feedback_value1 = analogRead(POTENTIOMETER_PIN1);
    pin1 = IN1_PIN;
    pin2 = IN2_PIN;
    potentimeter_pin = POTENTIOMETER_PIN1;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value = feedback_value2 = analogRead(POTENTIOMETER_PIN2);
    pin1 = IN3_PIN;
    pin2 = IN4_PIN;
    potentimeter_pin = POTENTIOMETER_PIN2;
  }
  //establish the move I shoul do
  if(feedback_value < position)
  {
    //this time I shoul extend the actuator
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else
  {
    //this time I should retract the actuator
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  feedback_value = analogRead(potentimeter_pin);
  //now I set the new position for feedback value for the moved actuator
  if(actuator == ACTUATOR1)
  {
    feedback_value1 = feedback_value;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value2 = feedback_value;
  }
}

// this function will check using forward kinematics 
// if the top of the arm is to close to the ground
// in this way I don't hit the claw to the ground
bool check_y(int position1, int position2)
{
  // I calculate the length of actuators
  double a1 = Al1 + map(position1, 28, 985, 0, 1000);
  double a2 = Al2 + map(position2, 28, 985, 0, 1000);
  // this termens are obtained by using math and will help me to calculate the y position
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