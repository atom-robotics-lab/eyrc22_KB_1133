#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class kb_naviagtion:

 def __init__(self):

  self.regions_= {
   'right': 0,
   'fright': 0,
   'front': 0,
   'fleft': 0,
   'left': 0
  }

  self.linear_velocity = 0.7
  self.kp = 1
  self._State = 0

  self.message = Twist

  rospy.init_node('kb_navigation')
  self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
  self.sub = rospy.Subscriber('ebot/laser/scan', LaserScan, self.clbk_laser)

 def clbk_laser(msg,self):

     laser_data = list(msg.ranges)
     for i in range(len(msg.ranges)):
         if(laser_data[i] <= 0.1):
             laser_data[i] = 100.0

     self.regions_ = {
         'right':  min(min(laser_data[0:106]), 8.0),
         'fright': min(laser_data[44], 10),
         'front':  min(min(laser_data[221:309]), 8.0),
         'fleft':  min(laser_data[134], 10),
         'left':   min(min(laser_data[434:531]), 8.0),
     }

     self.take_action()

 def move(self,linear,angular):

  velocity_msg = Twist()
  velocity_msg.linear.x = linear
  velocity_msg.angular.z = angular 
  self.pub_.publish(velocity_msg)

 def take_action(self):

  postion_error = self.regions_['left'] - self.regions_['right']

  if postion_error == 0 :
    self._State = 0 

  elif postion_error != 0  :

    self.angular_velocity = self.kp*postion_error
    self._State = 1


 def main():

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

    


if __name__ == '__main__':
 kb_nav = kb_naviagtion()
 kb_nav.move(kb_nav.linear_velocity , 0 )
 kb_nav.main()
 

