#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group1 = "arm"
        self._planning_group2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)

        # self._group.set_planning
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame2 = self._group2.get_planning_frame()
        self._eef_link2= self._group2.get_end_effector_link()
        self._group_names2 = self._robot.get_group_names()
        self._planning_frame1 = self._group1.get_planning_frame()
        self._eef_link1= self._group1.get_end_effector_link()
        self._group_names1 = self._robot.get_group_names()
        self._curr_state = self._robot.get_current_state()

        # Printing the planning frame , end effector link , group name 

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names1) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Go to predefined pose using the joint angles 

    def go_to_predefined_pose(self, arg_pose_name, group_id):

        group = None
        if group_id==1 :
            group = self._group1
        else:
            group = self._group2

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        group.set_named_target(arg_pose_name)
        plan = group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        try:
            goal.trajectory = plan[1]
        except:
           goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Go to function for predefined pose from moveit setup

    def go_to_pose(self, arg_pose):

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # self._group.setPlanningTime(10)
        self._group1.set_pose_target(arg_pose)
        # self._group1.setPlanningTime(10)
        flag_plan = self._group1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        plan = self._group1.plan()

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Setting up the joints angles

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group1.set_joint_value_target(arg_list_joint_angles)
        self._group1.plan()
        flag_plan = self._group1.go(wait=True)

        list_joint_values = self._group1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)   

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
    # calling ur5 class to go to the peppers

    ur5 = Ur5Moveit()

    # intermediate pose for red pepper
    intermediate_pose1 = [math.radians(88),math.radians(29),math.radians(-3),math.radians(-23),math.radians(0),math.radians(0)]
    #  intermediate pose for yellow pepper
    intermediate_pose2 = [math.radians(59),math.radians(39),math.radians(-71),math.radians(-23),math.radians(-17),math.radians(0)]
    # red pepper pose
    red_pepper = [math.radians(74),math.radians(59),math.radians(-90),math.radians(3),math.radians(1),math.radians(-9)]
    # red pepper drop pose
    red_drop = [math.radians(-31),math.radians(-7),math.radians(3),math.radians(-1),math.radians(-2),math.radians(-8)]
    # yellow pepper drop pose 
    yellow_drop = [math.radians(13),math.radians(-7),math.radians(3),math.radians(-1),math.radians(-2),math.radians(-8)]
    # yellow pepper pose
    yellow_pepper = [math.radians(94),math.radians(60),math.radians(-49),math.radians(0),math.radians(0),math.radians(0)]

    while not rospy.is_shutdown():  
        
        
        # ur5.set_joint_angles(intermediate_pose1) 
        ur5.set_joint_angles(intermediate_pose2) 
        ur5.set_joint_angles(red_pepper) 
           
        ur5.go_to_predefined_pose("closed", 2)
         
        ur5.set_joint_angles(intermediate_pose2) 
        ur5.set_joint_angles(red_drop) 

        ur5.go_to_predefined_pose("open", 2)

        #ur5.go_to_predefined_pose("close", 2)
        ur5.set_joint_angles(intermediate_pose1) 
        ur5.set_joint_angles(yellow_pepper) 
        ur5.go_to_predefined_pose("closed", 2)
        # ur5.set_joint_angles(intermediate_pose1) 
        ur5.set_joint_angles(yellow_drop)
        ur5.go_to_predefined_pose("open", 2)

        rospy.sleep(1)  

    del ur5

if __name__ == '__main__':
    main()