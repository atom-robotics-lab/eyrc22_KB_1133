#! /usr/bin/env python3

'''
ManipulationHelp Script - Hardware task - KB22

Author: Jaison Jose
Date  : 23 - 01 - 2023
Maintainer: ROS Team
Credit: e-Yantra, IIT Bombay
Copyright (c) 2022 e-Yantra IITB 

MoveIt class setup to avoid delay in initializing moveit 
group commander while performing tasks in remote network.

This script uses rostopics to communicate with the other 
manipulation nodes, so that the other nodes can:
-> publish joint angles and pose on specified topic
-> subscribe for flag acknowledgment (if setting pose/joint_angles is done)
-> subscribe for current joint angles and pose
'''


import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
from std_msgs.msg import Float32MultiArray, Bool, String, Empty
from geometry_msgs.msg import Pose


class Ur5Moveit_help:
    def __init__(self, pub_ee_pose_joint, pub_ee_pose_joint_wait_ack):
        '''
        Initializing UR5moveit class with robot, scene and group commander.
        - Planning_group: manipulator
        - wait_for_servers parameter of MoveGroupCommander determines the timeout for each:
            - Move_Action_Client
            - Move_Pick_Client
            - Move_Place_Client
            - Move_Execute_Client
        '''
        # Publishing topics from global
        self.pub_ee_pose_joint = pub_ee_pose_joint
        self.pub_ee_pose_joint_wait_ack = pub_ee_pose_joint_wait_ack

        # Initaializing moveit_commander
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        # wait_for_servers: 120s at 300ms ping KB to computer using IITB internet
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, wait_for_servers=120)

        # Initializing and waiting for execute trajectory ActionClient
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server() 

        # Storing necessary names of frame, link and group name
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Logging completion info
        rospy.loginfo("####################  KB22   ##########################") 
        rospy.loginfo("!!!!!!!!! MANIPULATION INITIALIZATION DONE !!!!!!!!!!!!") 
        rospy.loginfo("#######################################################") 

        # uncomment and it will log out initial current pose and joint values
        # self.print_pose_ee_and_joint_angles(None)

 
    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Function sets joint angles where wait is True. 
        Showing that the function will halt at the _group.go() command

        Input : std_msgs.msg.Float32MultiArray of joint values
        Output: Publishes acknowledgment of execution on /pose_joint_ack topic
        '''

        self._group.set_joint_value_target(arg_list_joint_angles.data)
        self._group.plan()
        flag_plan = self._group.go(wait=True) # Halts the program till the execution is returned with a status

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        self.pub_ee_pose_joint_wait_ack.publish(flag_plan) # publish acknowledgment of flag_plan
    

    def set_pose(self, arg_pose):
        '''
        Function sets joint angles where wait is True. 
        Showing that the function will halt at the _group.go() command
        
        Input : geometry_msgs.msg.Pose of pose values
        Output: Publishes acknowledgment of execution on /pose_joint_ack topic
        '''
        
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> set_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_pose() Failed." + '\033[0m')

        self.pub_ee_pose_joint_wait_ack.publish(flag_plan) # publish acknowledgment of flag_plan

    def print_pose_ee_and_joint_angles(self, _):
        '''
        Prints end-effector pose and joint angles of all the joints
        Publishes value in the form of string as:
        - [ quaternion_list, joint_angles_list]
        The same can be evaluated at the reciever end by using eval function in python
        '''
        list_pose_values = self._group.get_current_pose().pose
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + ">>> Current pose Values:" + '\033[0m')
        rospy.loginfo(list_pose_values) # Logs out current pose of the end-effector

        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values) # Logs out current joint angles of ur5

        list_pose_values = [list_pose_values.position.x, 
                            list_pose_values.position.y, 
                            list_pose_values.position.z, 
                            list_pose_values.orientation.x, 
                            list_pose_values.orientation.y, 
                            list_pose_values.orientation.z, 
                            list_pose_values.orientation.w]

        list_of_pose_joints = str([list_pose_values, list_joint_values])
        self.pub_ee_pose_joint.publish(list_of_pose_joints) # publishes in string format the current pose and joint values

    def __del__(self):
        '''
        class destructor:
            - Shutdown moveit_commander on the initialized node
            - logs the statement on the terminal
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("Object of class Ur5Moveit_help Deleted.")



def main():

    # # Global variables for class access
    # global pub_ee_pose_joint_wait_ack, pub_ee_pose_joint

    # Initializing this ros node
    rospy.init_node("Mani_Initializer", anonymous=True) # ensuring unique name

    # Publisher topic to give acknowledgment about 
    # the execution of joint and pose targets
    pub_ee_pose_joint_wait_ack = rospy.Publisher("/joint_pose_ack", 
                                                Bool, queue_size=1)

    # Publishing variable to know the position and joint angles
    pub_ee_pose_joint = rospy.Publisher("/joint_ee_pose", 
                                        String, queue_size=10)
    # Creating class instance
    ur5_help = Ur5Moveit_help(pub_ee_pose_joint, pub_ee_pose_joint_wait_ack)

    # Subscribing to joint angles in Float32MultiArray format
    rospy.Subscriber("/set_joint_value_target_wait_topic", 
                                            Float32MultiArray, ur5_help.set_joint_angles, 
                                            queue_size=10)

    # Subscribing to ee position values in Pose format
    rospy.Subscriber("/set_pose_value_target_wait_topic", 
                                            Pose, ur5_help.set_pose, 
                                            queue_size=10)

    # Subscribing to pose/joint req topic to publish ee_pose_joint values
    rospy.Subscriber("/joint_ee_pose_req", 
                        Empty, ur5_help.print_pose_ee_and_joint_angles, 
                        queue_size=1)

    # Spin the subscribers till the ros system is alive
    rospy.spin()


# Calling main if specified to start with script or 
# ignoring if called as module instance
if __name__ == "__main__":
    main()