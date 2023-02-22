#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class KB_Navigation:

    def __init__(self):
        self.pub_ = None
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
            'straight' : 0,
        }
        self.state_ = 0
        self.flag = False
        self.direction = -1 
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
            3 : 'turn right',
            4 : 'rotate_left',
            5 : 'stop',
            6 : 'follow left wall',
            7 : 'follow right wall'
        }

        self.message = Twist()
        
        rospy.init_node('Object_Avoider')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        self.sub = rospy.Subscriber('ebot/laser/scan', LaserScan, self.clbk_laser)
        self.pepper_found = rospy.Subscriber("/found", String, self.pepper_found_clbk)

        self.pepper_found_flag = False
       

        self.linear_p = 0.1         # Constant linear velocity between lanes
        self.angular_p = 0.25       # Angular velocity between lanes to keep in centre

        self.rotate_angular_p = 1   # Angular velocity while turning

        self.turn_flag = False      # Turn Flag
        self.turn_direction = 1     # Turn direction

        self.n_turns = 0            # No. of turns

        self.rotate_wall_dist = 0.5 # Distance from wall while turning

    def pepper_found_clbk(self, msg) :
        msg = msg.data

        try :
            if msg == "Stop" and self.pepper_found_flag == False :
                self.pepper_found_flag = True

                start = time.time()
                end = time.time()

                while(end-start < 3) :
                    print("Waiting for {} seconds".format(int(end-start)))
                    self.move(0, 0)
                    end = time.time()

                print("Start Moving")

            if msg == "Stop" and self.pepper_found_flag == True :
                print("Bot is already stopped")

            if msg == "Move" :
                self.pepper_found_flag = False

        except :
            print("Exception in pepper found callback !!")



    def clbk_laser(self, msg):
        
        laser_data = list(msg.ranges)
        
        for i in range(len(msg.ranges)):
            if(laser_data[i] <= 0.1):
                laser_data[i] = 100.0

        # Regions for Simulation
        '''self.regions = {
            'right':  min(min(msg.ranges[0:120]), 8), 
            'fright': min(min(msg.ranges[145:288]), 10), 
            'front':  min(min(msg.ranges[280:440]), 8), 
            'fleft':  min(min(msg.ranges[433:576]), 10), 
            'left':   min(min(msg.ranges[600:719]), 8),
            'straight' : min(min(msg.ranges[350:370]), 8)
        }'''

        # Regions for Hardware
        self.regions_ = {
        'right':  min(min(laser_data[0:106]), 8.0),
        'fright': min(laser_data[44], 10),
        'front':  min(min(laser_data[221:309]), 8.0),
        'fleft':  min(laser_data[134], 10),
        'left':   min(min(laser_data[434:531]), 8.0),
        }



        
        #print("\nLEFT : ", self.regions['left'], "FLEFT : ", self.regions['fleft'], "STRAIGHT : ", self.regions['straight'], "FRIGHT : ", self.regions['fright'], "RIGHT : ", self.regions['right'], "\n")

        self.take_action()
        

    def change_state(self, state):
        if state is not self.state_:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def move(self,linear,angular):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular 
        self.pub_.publish(velocity_msg)

    def take_action(self):

        try :

            '''if (self.regions['left'] < 0.9 and self.regions['right'] > 0.9) and (not self.turn_flag) :
                self.change_state(6)

            elif (self.regions['right'] < 0.9 and self.regions['left'] > 0.9) and (not self.turn_flag) :
                self.change_state(7)'''


            if ((self.regions['fright'] < 1 and self.regions['right'] < 1) or (self.regions['fleft'] < 1 and self.regions['left'] < 1)) and (not self.turn_flag):
                #print("Going Straight !!")
                self.change_state(2)


            elif self.regions['straight'] < 1.5 or self.turn_flag:
                self.turn_flag = True

                if self.n_turns != 2 :
                    self.change_state(1)

                    if (self.regions['fleft'] < 1 and self.regions['straight'] == 8) or (self.regions['fright'] < 1 and self.regions['straight'] == 8) :
                        print("Left Turn Completed !")
                        self.n_turns += 1
                        self.turn_flag = False
                else:
                    self.change_state(3)

                    if (self.regions['fleft'] < 1.3 and self.regions['straight'] == 8) or (self.regions['fright'] < 1.3 and self.regions['straight'] == 8) :
                        print("Right Turn Completed !")
                        self.n_turns += 1
                        self.turn_flag = False

            else:
                self.change_state(0)
                print("Else")

        except :
            print("Waiting for Laser Scan data !")

       

       

    def find_wall(self):
        self.move(self.linear_p , 0 )
        #print("Find Wall")

    def turn_right(self) :
        
        angular_error = self.regions['right'] - self.rotate_wall_dist
        self.move(0.1, -self.rotate_angular_p*angular_error) 
        print("Turn Right")

    def turn_left(self):

        angular_error = self.regions['left'] - self.rotate_wall_dist
        self.move(0.1, self.rotate_angular_p*angular_error) 
        print("Turn Left")
        #print("left angular error : ", self.rotate_angular_p*angular_error )

    

    def stop(self):
        self.move(0 ,0 )
        print("Bot Stopped !")

    def follow_the_wall(self):

        position_error = self.regions['left'] - self.regions['right']
        angular_velocity = self.angular_p*position_error

        if position_error > 0 :
            print("Drifting to left")
        elif position_error < 0 :
            print("Drifting to right")
        else:
            print("Going Straight !")

        self.move(self.linear_p, angular_velocity) 
        

    

    def follow_left_wall(self) :
        print("Following Left Wall ")
        
        position_error = self.regions['left'] - 0.7
        angular_velocity = self.angular_p*position_error
        #print("angular_velocity : ", angular_velocity)

        self.move(self.linear_p, angular_velocity) 

    def follow_right_wall(self) :
        print("Following Right Wall ")
        
        position_error = self.regions['right'] - 0.8
        angular_velocity = -self.angular_p*position_error

        self.move(self.linear_p, angular_velocity) 



    def _state_(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
    
            if self.state_ == 0:
                self.find_wall()
            elif self.state_ == 1:
                self.turn_left()
            elif self.state_ == 2:
                self.follow_the_wall()
            elif self.state_ == 3 :
                self.turn_right()
            elif self.state_ == 4 :
                self.rotate_left()
            elif self.state_ == 5 :
                self.stop()
            elif self.state_ == 6 :
                self.follow_left_wall()
            elif self.state_ == 7 :
                self.follow_right_wall()
            else:
                rospy.logerr('Unknown state!')
                        
            rate.sleep()



if __name__ == '__main__':
    obs_state = KB_Navigation()

    #obs_state.start_move()
    #rospy.sleep(2)

    obs_state._state_()