#!/usr/bin/env python

import sys
import rospy
import serial
import io

from time import sleep
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from copy import copy, deepcopy

from geometry_msgs.msg import Twist, TwistWithCovariance


class Command:
   isAlive = False   # Set to True if subscrived command message has been received
   mode = 0          # Command mode (0:vel, rot) <--> (1:speedL, speedR)
   speed = 0.0       # Speed mm/s
   deg_sec = 0.0     # Rotational speed deg/s
   speedL = 0.0      # Left Wheel speed mm/s
   speedR = 0.0      # Right wheel speed mm/s

class VehicleConfig(object):
   BodyCircumference = 0   # circumference length of robot for spin in place
   WheelCircumference = 0
   WIDTH = 0.0             # Default Vehicle width in mm
   WHEEL_R = 0.0           # Wheel radius
   WHEEL_MAXV = 0.0        # Maximum wheel speed (mm/s)
   V_Limit = 0             # Speed limit for vehicle (m/s)
   W_Limit = 0             # Rotational Speed limit for vehicle (rad/s)


class Robot:
   rospy.init_node('omoros', anonymous=True)
   # fetch /global parameters
   param_port = rospy.get_param('~port')
   param_baud = rospy.get_param('~baud')
   param_modelName = rospy.get_param('~modelName')
   ser = serial.Serial(param_port, param_baud)
   config = VehicleConfig()
   ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                           newline = '\r',
                           line_buffering = True)
   cmd = Command
   vel = 0.0            # Velocity returned from CVW command
   rot = 0.0            # Rotational speed returned from CVR command

   def __init__(self):
      ## Set vehicle specific configurations
      if self.param_modelName == "r1":
         print "**********"
         print "Driving R1"
         print "**********"
         self.config.WIDTH = 0.591        # Apply vehicle width for R1 version
         self.config.WHEEL_R = 0.11       # Apply wheel radius for R1 version
         self.config.WHEEL_MAXV = 1200.0  # Maximum speed can be applied to each wheel (mm/s)
         self.config.V_Limit = 0.6        # Limited speed (m/s)
         self.config.W_Limit = 0.1

      else :
         print "Error: param:modelName, Only support r1"
         exit()

      print('Serial port:'+self.ser.name)         # Print which port was really used

      if self.ser.isOpen():
         print("Serial Open")         

      rospy.Subscriber("cmd_vel", Twist, self.callbackCmdVel)
      
      
      rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz


      while not rospy.is_shutdown():
         if self.cmd.isAlive == True:
             self.cmd.cnt += 1
             if self.cmd.cnt > 1000:         #Wait for about 3 seconds 
                 self.cmd.isAlive = False
         rate.sleep()
         
      self.ser.close()

         
   def callbackCmdVel(self, cmd):
      print "CMD_VEL: {:.2f} {:.2f} ".format(cmd.linear.x, cmd.angular.z) #was comment
      cmdV = cmd.linear.x
      cmdW = cmd.angular.z
      if cmdV>self.config.V_Limit:
         cmdV = self.config.V_Limit
      elif cmdV<-self.config.V_Limit:
         cmdV = -self.config.V_Limit
      if cmdW>self.config.W_Limit:
         cmdW = self.config.W_Limit
      elif cmdW<-self.config.W_Limit:
         cmdW = -self.config.W_Limit
      (speedL,speedR) = self.getWheelSpeedLR(self.config, cmdV, cmdW)
      print "SPEED LR: {:.2f} {:.2f} ".format(speedL, speedR) #was comment
      self.sendCDIFFVcontrol(speedL*200, speedR*200)

   def sendCVWcontrol(self, config, V_mm_s, W_mrad_s):
      if V_mm_s > config.V_Limit :
         V_mm_s = config.V_Limit
      elif V_mm_s < -config.V_Limit :
         V_mm_s = -config.V_limit
      if W_mrad_s > config.W_Limit :
         W_mrad_s = config.W_Limit
      elif W_mrad_s < -config.W_Limit:
         W_mrad_s = -config.W_Limit
      # Make a serial message to be sent to motor driver unit
      cmd = '$CVW,{:.0f},{:.0f}'.format(V_mm_s, W_mrad_s)
      print cmd
      if self.ser.isOpen():
         print cmd
         self.ser.write(cmd+"\r"+"\n")

   def sendCDIFFVcontrol(self, VLmm_s, VRmm_s):
      """ Set differential wheel speed for Left and Right """
      if VLmm_s > self.config.WHEEL_MAXV :
         VLmm_s = self.config.WHEEL_MAXV
      elif VLmm_s < -self.config.WHEEL_MAXV :
         VLmm_s = -self.config.WHEEL_MAXV
      if VRmm_s > self.config.WHEEL_MAXV :
         VRmm_s = self.config.WHEEL_MAXV
      elif VRmm_s < -self.config.WHEEL_MAXV :
         VRmm_s = -self.config.WHEEL_MAXV
      # Make a serial message to be sent to motor driver unit
      cmd = '$CDIFFV,{:.0f},{:.0f}'.format(VLmm_s, VRmm_s)
      if self.ser.isOpen():
         print cmd
         self.ser.write(cmd+"\r"+"\n")

   def getWheelSpeedLR(self, config, V_m_s, W_rad_s):
      speedL = V_m_s - config.WIDTH * W_rad_s / config.WHEEL_R /2.0
      speedR = V_m_s + config.WIDTH * W_rad_s / config.WHEEL_R /2.0
      return speedL, speedR
        
if __name__ == '__main__':

   try:
      Robot()
   except rospy.ROSInterruptException:
      pass
    
   


