#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Object_Avoider:

    def __init__(self):
        self.pub_ = None
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.state_ = 2
        self.flag = False
        self.direction = -1 
        self.state_dict_ = {
            0: 'find the wall',
            1: 'turn left',
            2: 'follow the wall',
            3 : 'turn right',
            4 : 'rotate_left'
        }

        self.message = Twist()
        self.d = 0.7
        self.flag = 0 
        self.rotation1 = 1.6
        self.portion = "right"
        rospy.init_node('Object_Avoider')
        self.pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)        
        self.sub = rospy.Subscriber('ebot/laser/scan', LaserScan, self.clbk_laser)

        self.flag_turn1 = True
        self.flag_follow_wall = False
        self.flag_initial_start = True
        self.flag_turning1 = False

        self.flag_turn2 = True
        self.flag_turning2 = False

        self.flag_turn2_stage = 0


    def clbk_laser(self, msg):
        
        self.regions = {
            'right':  min(min(msg.ranges[0:120]), 10), 
            'fright': min(min(msg.ranges[145:288]), 10), 
            'front':  min(min(msg.ranges[280:440]), 10), 
            'fleft':  min(min(msg.ranges[433:576]), 10), 
            'left':   min(min(msg.ranges[600:719]), 10),
            'straight' : min(min(msg.ranges[350:370]), 10)
        }

        #print("FRONT     : ", self.regions['front'])
        print("LEFT : ", self.regions['left'], "STRAIGHT : ", self.regions['straight'], "RIGHT : ", self.regions['right'])
        #print("RIGHT : ", self.regions['right'])

        if self.regions['left'] < 2 and self.regions['right'] <2 :
            self.flag_follow_wall = True

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

        if self.regions['left'] > 2 and self.regions['right'] > 2 and self.flag_initial_start:
            print("Initial Start")
            self.change_state(2)       

        elif (self.regions['front'] < 1.2 and (self.flag_turn1 and (not self.flag_initial_start))) or self.flag_turning1 : # and self.flag_follow_wall):
            self.flag_turning1 = True
            #rospy.loginfo("end of first lane")
            #rospy.loginfo(self.regions)
            self.change_state(1)

            if (self.regions['left'] > 0.5) : # and self.regions['front'] < 2.86) :
                #print("entered while loop")            
                #rospy.loginfo(self.regions)
                self.change_state(1)
 
            if  (self.regions['front'] > 2.1 and (self.regions['left'] < 0.67 or self.regions['fleft'] < 0.6)) or (self.regions['straight'] > 5):
                self.flag_turn1 = False
                self.flag_turning1 = False

        elif ((self.regions['left'] > 1.7 and self.regions['straight'] < 0.9) and ((not self.flag_turning1) and (not self.flag_turn1) )) and (self.flag_turn2 and (not self.flag_initial_start)) or self.flag_turning2:
            self.flag_turning2 = True
            #print("Second left turn")

            if self.flag_turn2_stage == 0 :
                print("Stage - 0")

                self.change_state(4)
            
                if self.regions['straight'] > 5 and self.flag_turn2_stage == 0:
                    self.flag_turn2_stage = 1
                    
                    #self.flag_turn2 = False
                    #self.flag_turning2 = False
                    #print("EXITING ROTATE LEFT")

            elif self.flag_turn2_stage == 1 :
                print("Stage - 1")
                
                self.change_state(2)

                if self.regions['straight'] < 2.3 :
                    self.flag_turn2_stage = 2

            elif self.flag_turn2_stage == 2 :
                print("Stage - 2")
                self.change_state(4)

                if self.regions['straight'] > 5:
                    self.flag_turn2_stage = 3

            elif self.flag_turn2_stage == 3 :
                self.change_state(2)

                if self.regions['left'] < 0.8 :

                    
                    self.change_state(2)
                    self.flag_turn2 = False
                    self.flag_turning2 = False


        elif (self.regions['left'] < 0.4 or self.regions['fleft'] < 0.4) and ((not self.flag_turn1) and (not self.flag_turning1)):
            self.turn_right()
            #print("RIGHT")
            #self.change_state(2)
        
        elif self.regions['left'] > 0.73 and ((not self.flag_turn1) and (not self.flag_turning1)) and self.regions['straight'] > 1.63:
            self.turn_left_minor()
            #print("RIGHT")
            #self.change_state(2)
        else:
            self.change_state(2)#
            #rospy.loginfo(self.regions)
            #rospy.loginfo("Detected no wall")

       

    def find_wall(self):
        self.move(0 , 0 )
        print("Find Wall")

    def turn_right(self) :
        self.move(0.5, - 1)
        print("Turn Right")

    def turn_left(self):
        self.move(0.6,1.65) #0.1
        print("Turn Left")
        print(self.direction)

    def turn_left_minor(self) :
        self.move(0.5, 1)
        print("Minor Left")

    def stop(self):
        self.move(0 ,0 )
        print("Stop")

    def follow_the_wall(self):
        
        print("Follow Wall")
        self.move(0.6, 0) #0.3
        self.flag_initial_start = False
        #print(self.direction)  

    def start_move(self) :
        self.move(1 ,0 )
        #rospy.sleep(5)

    def rotate_left(self) :
        print("Rotating Left")
        self.move(0, 1 )


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
            else:
                rospy.logerr('Unknown state!')
                        
            rate.sleep()



if __name__ == '__main__':
    obs_state = Object_Avoider()

    obs_state.start_move()
    #rospy.sleep(2)

    obs_state._state_()