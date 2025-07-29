import sys
import os
sys.path.append(os.path.relpath("../"))

from math import pi as PI, sqrt, atan
import math
import time
import threading

from help.manager import Manager
from help.help import getTime
from define.define import *

import rospy
from sensor_msgs.msg import LaserScan

class CarState:
    MANUAL = 0
    AUTONOMOUS = 1

class AutonomousState:
    EXPLORING = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    FORWARD = 3
    GRABBING = 4

class Car:
    AUTO_PWM = 20
    REVERSE_MILLIS = 1000
    TURNING_MILLIS = 1500

    CENTER_THRESH_X = 50
    CENTER_THRESH_Y = 50

    FORWARD_DIST = 1
    GRAB_TIMEOUT = 10000

    OBSTACLE_DISTANCE_THRESHOLD = 1.0
    
    def scan_callback(self,msg):
        self.latest_scan = msg

    def __init__(self, processManager: Manager, motors, Srobot, move_base_cmd, server_robot):
        self.state = CarState.MANUAL
        self.autoState = AutonomousState.EXPLORING

        self._processManager = processManager
        self._motors = motors
        self._robot = robot
        self._move_base_cmd = move_base_cmd
        self._server_robot = server_robot

        self.emergency()
        self.endEmergency()
        self.arduinoState = MotorStates.IDLE

        self.timerEnd = 0
        self.grabTimer = 0
        self._avoidance_thread = None
        self.latest_scan = None
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        

    def start(self):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.START])

    def emergency(self):
        self._processManager.sendMessageTo(self._motors, [0, ArduinoCommands.EMERGENCY])

    def endEmergency(self):
        self._processManager.sendMessageTo(self._motors, [0, ArduinoCommands.END_EMERGENCY])

    def brake(self):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.BREAK])

    def forward(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 0, 0])

    def backward(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 1, 1])

    def turnLeft(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 0, 1])

    def turnRight(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 1, 0])

    def cancel_goal(self):
        self._processManager.sendMessageTo(self._move_base_cmd, ["cancel_goal", []])

    def send_goal(self, args):
        self._processManager.sendMessageTo(self._move_base_cmd, ["send_goal", args])

    def get_position(self):
        self._processManager.sendMessageTo(self._move_base_cmd, ["get_position", []]) 
        return self._processManager.getMessageFrom(self._move_base_cmd, 5)

    def getDistance(self):
        front_ranges = scan.ranges[0:30] + scan.ranges[-30:]
        dists = [d for d in front_ranges if d > 0.05]

        if not dists:
            return float('inf')
        return min(dists)
        

    def autonomousAvoidanceLoop(self):
        print("Autonomous avoidance loop started")
        print("Autonomous avoidance loop started")
        self._running = True
        while self.state == CarState.AUTONOMOUS and self._running:
       
            distance = self.getDistance()
            if distance is None:
                print("No distance received.")
                self.forward(self.AUTO_PWM)
            elif distance < Car.OBSTACLE_DISTANCE_THRESHOLD:
                print(f"Obstacle detected at {distance:.2f}m. Turning right.")
                self.turnRight(self.AUTO_PWM)
                time.sleep(0.5)
            else:
                print(f"Path clear. Moving forward.")
                self.forward(self.AUTO_PWM)
            time.sleep(0.2)

    def startAutonomousAvoidance(self):
        self.setAutonomous()
        self.endEmergency()
        self.start()
        self.autoState = AutonomousState.EXPLORING
        if self._avoidance_thread is None or not self._avoidance_thread.is_alive():
            self._avoidance_thread = threading.Thread(target=self.autonomousAvoidanceLoop, daemon=True)
            self._avoidance_thread.start()

    def setAutonomous(self):
        self.state = CarState.AUTONOMOUS
        self.autoState = AutonomousState.EXPLORING
        self.arduinoState = MotorStates.RUNNING

    def setManual(self):
        self.state = CarState.MANUAL
        self.emergency()
        self.arduinoState = MotorStates.EMERGENCY
    
    def set_pwm(self, left_pwm,right_pwm):
        msg = [0,4,left_pwm,right_pwm]
        self.pipe.send(msg)

    def update(self, objData, secondCameraObj):
      if self.state == CarState.MANUAL:
          print("MANUAL")
          self.cancel_goal()
          return
  
      now = getTime()
      
    
   
      # Timeout for GRABBING
      if self.autoState == AutonomousState.GRABBING:
          if now - self.grabTimer > Car.GRAB_TIMEOUT:
              self.autoState = AutonomousState.EXPLORING
              print("Timeout GRABBING -> EXPLORING")
              self.cancel_goal()
          return
  
      # GRABBING logic triggered by second camera
      if secondCameraObj is not None and self.autoState != AutonomousState.GRABBING:
          self.autoState = AutonomousState.GRABBING
          print("GRABBING")
          self.cancel_goal()
          self.grabTimer = now
          self.armCoordsFromBox(secondCameraObj)
          return
      
      if self.latest_scan and selr.auto:
          dist = getDistance(self.latest_scan)
          print(f" Dist min in fata: {dist:.2f} m")
          
          if dist >1.0:
            pwm = 30
            self.set_pwm(pwm,pwm)
            
          else:
            pwm = 30
            self.set_pwm(pwm,-pwm)
          
  
      # Navigation logic via objData (car_camera)
      if objData is not None:
          x, y = objData[0][0][0]
  
          if x < -Car.CENTER_THRESH_X and self.autoState != AutonomousState.TURN_LEFT:
              self.autoState = AutonomousState.TURN_LEFT
              print("TURN_LEFT")
              pos = self.get_position()
              if pos:
                  self.send_goal([pos[0], pos[1], pos[2] + PI / 2])
              return
  
          elif x > Car.CENTER_THRESH_X and self.autoState != AutonomousState.TURN_RIGHT:
              self.autoState = AutonomousState.TURN_RIGHT
              print("TURN_RIGHT")
              pos = self.get_position()
              if pos:
                  self.send_goal([pos[0], pos[1], pos[2] - PI / 2])
              return
  
          elif self.autoState != AutonomousState.FORWARD:
              self.autoState = AutonomousState.FORWARD
              print("FORWARD")
              pos = self.get_position()
              if pos:
                  goal = [
                      pos[0] + math.cos(pos[2]) * Car.FORWARD_DIST,
                      pos[1] + math.sin(pos[2]) * Car.FORWARD_DIST,
                      pos[2]
                  ]
                  self.send_goal(goal)
              return

    # No object in any camera � explore
      if objData is None and secondCameraObj is None:
          if self.autoState != AutonomousState.EXPLORING:
              self.autoState = AutonomousState.EXPLORING
              print("EXPLORING")
              self.cancel_goal()
              pos = self.get_position()
              if pos:
                  goal = [
                      pos[0] + math.cos(pos[2]) * 0.4,
                      pos[1] + math.sin(pos[2]) * 0.4,
                      pos[2]
                  ]
                  self.send_goal(goal)




    def setAutonomous(self):
        self.state = CarState.AUTONOMOUS
        self.autoState = AutonomousState.EXPLORING

        #self.endEmergency()
        #self.start()
        self.arduinoState = MotorStates.RUNNING

    def setManual(self):
        self.state = CarState.MANUAL
        self.emergency()
        self.arduinoState = MotorStates.EMERGENCY
        # Opre?te thread-ul de autonomie
        if self._avoidance_thread and self._avoidance_thread.is_alive():
            self._running = False
            self._avoidance_thread.join()

    def armCoordsFromBox(self, response):
        x1,y1 = response[0][1][0]
        x2,y2 = response[0][1][1]
        # these are some constants that are mesured
        l1 = 67 * 100 # lenght of the vertical link
        l2 = 107 * 100 # lenght of the horizontal link
        pixelToMicroM = 6.875 # how much is a pixel in hundred micrometers
        heightCamera = 54 # the distance from the camera to the ground
        # determine the center of the bounding box
        xCameraPixel = (x1 + x2) / 2
        yCameraPixel = (y1 + y2) / 2
        # transform pixel coordinates to real world coordinates
        xCameraInMicroM=int(xCameraPixel*pixelToMicroM)
        yCameraInMicroM=int((480 - yCameraPixel) * pixelToMicroM)
        # because they are relative to center of the camera
        # I move the coordinates relative to bittom join
        xFinal = yCameraInMicroM + sqrt(l2 * l2 - (l1 - heightCamera) * (l1 - heightCamera))
        yFinal = 0 
        # deterime how much I need to rotate the arm00
        alpha_real = abs(atan(abs(((320 - yCameraPixel) * pixelToMicroM / xFinal))) * 180 / (PI))
        if alpha_real - int(alpha_real) >= 0.4:
            alpha = int(alpha_real) + 2
        else:
            alpha = int(alpha_real)

        #print(f"x:{xFinal} alpha:{alpha}")
        xFinal = xFinal + 200
        if xFinal > 10600:
            print("Obj detected but not in range of arm")
            return

        area = (x2 - x1) * (y2 - y1)
        # determie how much I need to rotate the claw
        beta = atan((x2 - x1) / (y2 - y1)) * 180 / PI
        if beta > 15 or beta < 75:
            # send command to rotate the claw
            receiver = self._robot

            self._processManager.sendMessageTo(self._robot, [0, ArduinoCommands.END_EMERGENCY])

            message = [0, RobotCommands.ROTATE_CLAW, int(beta)]
            self._processManager.sendMessageTo(receiver, message)
            t = getTime()
            while getTime() - t < 1000:
                pass
            #get new bounding box
            response = self._processManager.getMessageFrom(self._server_robot)
            t = getTime()
            while response == None and getTime() - t <= 2000:
                response = self._processManager.getMessageFrom(self._server_robot)
            if response != None:
                x1,y1 = response[0][1][0]
                x2,y2 = response[0][1][1]
            # if the bounding box area is not modified then 
            # the claw has rotated to the needed angle or if the respone is None
            # the the claw cover the bottle
            area2 = (x2 - x1) * (y2 - y1)
            print(f"1: {area}, 2:{area2}")
            if response == None or area2 < area * 0.85 or area2 > 1.15 * area:
                beta += 90
        elif beta <= 20:
            beta = 0
        else:
            beta = 90
        # I determine the direction I need to rotate the arm
        if xCameraPixel > 320:
            dir = 1
        else:
            dir = 0
        print(f"xFinal = {xFinal}, yFinal = {yFinal}, alpha = {alpha}, beta = {beta}, xCameraMicroM = {xCameraPixel}, yCamera = {yCameraInMicroM}, dir = {dir} alpha_real = {alpha_real}")
        #print(f"x1 = {x1}, y1 = {y1}, x2 = {x2}, y2 = {y2}")
        
        #return
        
        self._processManager.sendMessageTo(self._robot, [0, ArduinoCommands.END_EMERGENCY])

        alpha = 0
        message = [0, RobotCommands.FINAL_ARM, dir, alpha, int(xFinal), 0, int(beta)]
        self._processManager.sendMessageTo(self._robot, message)
        pass
