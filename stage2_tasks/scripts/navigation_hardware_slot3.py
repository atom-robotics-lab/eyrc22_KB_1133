#! /usr/bin/env python3


''' 
* Team Id : KB#1133
* Author List : Arjun K Haridas, Ayan Goel, Bhavay Garg>
* Filename: navigation_hardware_slot3.py
* Theme: Krishi Bot
* Functions: __init__, joint_move_clbk, arm_feedback, pepper_found_clbk, clbk_laser, change_state, move, take_action, find_wall, turn_right, turn_left, stop, 
             follow_the_wall, follow_left_wall, follow_right_wall, _state_
* Global Variables: none
'''

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class KB_Navigation:
    def __init__(self):
        
        self.pub_ = None
        self.arm_pub = None
        
        # Convert LiDAR values to various regions
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
            'straight' : 0,
        }
        
        # Define the current state of the bot
        self.state_ = 0

        # Define the various states of the bot
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

        # Create empty Twist message
        self.message = Twist()
        
        # Create a rospy node
        rospy.init_node('Object_Avoider')
        
        # Publisher to topic /cmd_vel
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Publisher to topic /arm_rotation
        self.arm_pub = rospy.Publisher('/arm_rotation', String, queue_size = 1)    
        
        # Subscriber to topic /ebot/laser/scan
        self.sub = rospy.Subscriber('ebot/laser/scan', LaserScan, self.clbk_laser)

        # Subscriber to topic /found
        self.pepper_found = rospy.Subscriber("/found", String, self.pepper_found_clbk)

        # Subscriber to topic /joint_function
        self.pluck_sub = rospy.Subscriber("/joint_function" , String , self.joint_move_clbk)

        # Flag variable to indicate if pepper is found
        self.pepper_found_flag = False
       
        # Constant linear velocity between lanes
        self.linear_p = 0.05  

        # Angular velocity between lanes to keep in centre       
        self.angular_p = 0.25       

        # Angular velocity while turning
        self.rotate_angular_p = 1   

        # No. of turns taken by the bot
        self.n_turns = 0    

        # Distance from wall while turning   
        self.rotate_wall_dist = 0.8 

        # Used to stop the bot when pepper is detected
        self.stop_counter = 0

    def joint_move_clbk(self , msg):
        '''
        * Function Name: joint_move_clbk
        * Input: msg(Can contain certain discrete values like "Stop" and "Move")
        * Output: none
        * Logic: This is a callback function for the Topic : /joint_function.
                It is used to keep the bot stopped when the arm is trying to pick the fruit and then 
                later start the movement when the arm is done placing the peppper in the basket
        * Example Call: Callback Function
        '''
        msg_data = msg.data 
        try : 
            print("MSG DATA : ", msg_data)
            if msg_data == "Stop" :
                self.stop_counter = 1 
            elif msg_data == "Move" :
                self.stop_counter = 0 
            else : 
                self.stop_counter = 0
        except: 
            print("Not even a single value is published")

    def arm_feedback(self) :
        '''
        * Function Name: arm_feedback
        * Input: none
        * Output: none
        * Logic: This function is used to publish the message "Rotate" to the
                 /arm_rotation topic. It is used to rotate the direction of the arm(left-facing
                 or right-facing) after the turn-2(when the bot enters the center lane again)

        * Example Call: arm_feedback()
        '''        
        self.arm_pub.publish("Rotate")

    def pepper_found_clbk(self, msg) :
        '''
        * Function Name: pepper_found_clbk
        * Input: msg("Can contain discrete values like "Stop" and "Move")
        * Output: none
        * Logic: This is a callback function for the topic /found. It will stop the bot 
                 when a pepper is detected by the perception script

        * Example Call: Callback Function
        '''  
        msg = msg.data

        try :
            if msg == "Stop" and self.pepper_found_flag == False :
                self.pepper_found_flag = True
                self.stop_counter == 1 

            if msg == "Stop" and self.pepper_found_flag == True :
                print("Bot is already stopped")

            if msg == "Move" :
                self.pepper_found_flag = False

        except :
            print("Exception in pepper found callback !!")



    def clbk_laser(self, msg):
        '''
        * Function Name: clbk_laser
        * Input: msg(Contains LaserScan data)
        * Output: none
        * Logic: This function is used to receive data from the LiDAR and divide it into 
                 various segments like front, left, right etc.

        * Example Call: Callback Function
        '''  
        
        laser_data = list(msg.ranges)
        
        for i in range(len(msg.ranges)):
            if(laser_data[i] <= 0.1):
                laser_data[i] = 100.0

        # LiDAR Regions for Hardware
        self.regions = {
        'right':  min(min(laser_data[0:106]), 8.0),
        'fright': min(laser_data[44], 10),
        'front':  min(min(laser_data[221:309]), 8.0),
        'fleft':  min(laser_data[134], 10),
        'left':   min(min(laser_data[434:531]), 8.0),
        'straight' : min(min(laser_data[261:269]), 8.0)
        }
        
        self.take_action()
        

    def change_state(self, state):
        '''
        * Function Name: change_state
        * Input: state(Numerical values between 0 and 7)
        * Output: none
        * Logic: This function is used to change the state of the navigation script. 
                 Different integers is mapped to different states of the bot

        * Example Call: change_state(0)
        '''  
        if state is not self.state_:
            print ('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state

    def move(self,linear,angular):
        '''
        * Function Name: move
        * Input: linear(linear velocity), angular(angular velocity)
        * Output: none
        * Logic: This function takes the linear and angular velocities as input
                 and sends velocity message to the /cmd_vel topic

        * Example Call: move(0, 0)
        '''  
        velocity_msg = Twist()
        velocity_msg.linear.x = linear
        velocity_msg.angular.z = angular 
        self.pub_.publish(velocity_msg)

    def take_action(self):
        '''
        * Function Name: take_action
        * Input: none
        * Output: none
        * Logic: This function is the main controlling unit of the navigation script.
                 It uses the LiDAR values to decide which step to take(follow the wall
                 , stop the bot etc.) 
        * Example Call: take_action() 
        '''  

        try :
            
            # Stop the bot if stop_counter == 1
            if self.stop_counter == 1 :
                self.change_state(5)

            else :
                
                # Travel between the troughs, while keeping the bot at the center
                if ((self.regions['fright'] < 1.2 and self.regions['right'] < 1) or (self.regions['fleft'] < 1.2 and self.regions['left'] < 1)) and (not self.turn_flag):
                    self.change_state(2)
                
                # Travel between the troughs
                elif (self.regions['fright'] < 2.8 and self.regions['fleft'] < 1.2) and (not self.turn_flag) :
                    self.change_state(2)

                # Turn the bpt
                elif (self.regions['straight'] < 1.5 or self.turn_flag ):
                    self.turn_flag = True
                    print("stop counter" , self.stop_counter)

                    # If the no. of turns till now if less than 2, turn left
                    if self.n_turns < 2 :
                        self.change_state(1)

                        if (self.regions['fleft'] < 1 and self.regions['straight'] >=7) or (self.regions['fright'] < 1 and self.regions['straight'] >= 7) :
                            print("Left Turn Completed !")
                            self.n_turns += 1
                            self.turn_flag = False

                            # Rotate the direction of the arm
                            if self.n_turns == 2 :
                                print("Arm Feedback !!")
                                self.arm_feedback()
                                
                    # Take the right turn
                    else:
                        self.change_state(3)

                        if (self.regions['fleft'] < 1.3 and self.regions['straight'] >=7) or (self.regions['fright'] < 1.3 and self.regions['straight'] >=7) :
                            print("Right Turn Completed !")
                            self.n_turns += 1
                            self.turn_flag = False

                # Find the wall
                else:
                    self.change_state(0)
                    print("Else")

        except :
            print("Waiting for Laser Scan data !")

       

       

    def find_wall(self):
        '''
        * Function Name: find_wall
        * Input: none
        * Output: none
        * Logic: This function will make the bot travel in a straight line ahead
                 with a constant velocity until a wall is detected

        * Example Call: find_wall()
        '''  
        self.move(self.linear_p , 0 )

    def turn_right(self) :
        '''
        * Function Name: turn_right
        * Input: none
        * Output: none
        * Logic: This function is used to make the bot take a right turn
        * Example Call: turn_right()
        '''  
        
        angular_error = self.regions['right'] - self.rotate_wall_dist
        self.move(0.1, -self.rotate_angular_p*angular_error) 
        print("Turn Right")

    def turn_left(self):
        '''
        * Function Name: turn_left
        * Input: none
        * Output: none
        * Logic: This function is used to make the bot take a left turn
        * Example Call: turn_left()
        '''  

        angular_error = self.regions['left'] - self.rotate_wall_dist
        self.move(0.1, self.rotate_angular_p*angular_error) 
        print("Turn Left")

    

    def stop(self):
        '''
        * Function Name: stop
        * Input: none
        * Output: none
        * Logic: This function is used to make the bot stop when a pepper is detected
        * Example Call: stop()
        '''  
        self.move(0 ,0 )
        print("Bot Stopped !")

    def follow_the_wall(self):
        '''
        * Function Name: follow_the_wall
        * Input: none
        * Output: none
        * Logic: This function is used to make the bot navigate between the troughs
                 while staying at the center of the lane
        * Example Call: follow_the_wall()
        '''  

        position_error = self.regions['left'] - self.regions['right']
        angular_velocity = self.angular_p*position_error

        if position_error > 0 :
            print("Drifting to left")
        elif position_error < 0 :
            print("Drifting to right")
        else:
            print("Going Straight !")

        self.move(self.linear_p, angular_velocity)    

    
    def _state_(self):
        '''
        * Function Name: _state_
        * Input: none
        * Output: none
        * Logic: This function is used to change the state of the bot.
                It is called by the change_state() function

        * Example Call: Callback Function
        '''  

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
                print("Sim function")
            elif self.state_ == 5 :
                self.stop()
            else:
                rospy.logerr('Unknown state!')
                        
            rate.sleep()



if __name__ == '__main__':
    obs_state = KB_Navigation()
    obs_state._state_()