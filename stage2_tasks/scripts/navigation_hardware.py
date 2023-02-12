#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class kb_navigation:
    def __init__(self):
        self.regions_= {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0
        }

        
        self.linear_p = 0.5
        self.angular_p = 0.1


        self._State = 0
        self.linear_velocity = 0.5

        self.message = Twist

        rospy.init_node('kb_navigation')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        self.sub = rospy.Subscriber('ebot/laser/scan', LaserScan, self.clbk_laser)
        self.pepper_found = rospy.Subscriber("/found", String, self.pepper_found_clbk)

        self.pepper_found_flag = True

    def pepper_found_clbk(self, msg) :
        msg = msg.data

        if msg == "Stop" and self.pepper_found_flag == False:
            self.pepper_found_flag = True        	
            start = time.time()
            end = time.time()

            while (end-start < 2) :      
                print("Waiting for {} seconds".format(end-start))              
                self.move(0, 0)
                end = time.time()
            print("Start Moving")
            
        if msg == "Stop" and self.pepper_found_flag == True:
            print("Bot is already stopped")
        
        if msg == "Move" :
            self.pepper_found_flag = False
            




    def clbk_laser(self, msg):
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

        #print("callback")

        '''self.regions_ = {
            'right':  min(min(msg.ranges[0:120]), 8), 
            'fright': min(min(msg.ranges[145:288]), 8), 
            'front':  min(min(msg.ranges[280:440]), 8), 
            'fleft':  min(min(msg.ranges[433:576]), 8), 
            'left':   min(min(msg.ranges[600:719]), 8),
            'straight' : min(max(msg.ranges[350:370]), 8)
        }'''

        self.take_action()

    def move(self,linear,angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular 
        self.pub_.publish(velocity_msg)

    def take_action(self):

        try :
            if self.regions_['front'] < 1.0 :
                print("Stop!! Wall in front")
                self.move(0, 0)

            elif self.regions_['left'] > 1 and self.regions_['right'] > 1 :
                print("Go Straight, no trough found")
                self.move(self.linear_p, 0)        

            else :

                position_error = self.regions_['left'] - self.regions_['right']
                #print("Left : ", self.regions_['left'])

                if position_error == 0 :
                    print("Go Straight")
                    self.move(self.linear_p, 0)
                    #self._State = 0 

                else :
                    if position_error>0 :
                        print("Turning Left")
                    else:
                        print("Turning Right")

                    self.angular_velocity = self.angular_p*position_error
                    self.move(self.linear_p, self.angular_velocity)

        except :
            print("Waiting for Laser Scan data")
                    


'''def main():

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():'''

    


if __name__ == '__main__':
    kb_nav = kb_navigation()
    kb_nav.take_action()
    rospy.spin()
 