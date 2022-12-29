#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 1133 ]
# Author List:		[ Arjun K Haridas, Bhavay Garg , Ayan Goel , Divyansh Sharma ]
# Filename:			percepStack.py
# Functions:		[ mask, img_clbk, depth_clbk, image_processing, main ]


####################### IMPORT MODULES #######################
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import sys,math
import tf
from std_msgs.msg import String

##############################################################



class ManiStack:

    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self._planning_group1 = "arm"
        self._planning_group2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)

        self.pub_tf2 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf3 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.pluck_pub=rospy.Publisher('/pluck_pub', String, queue_size = 1)
        
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
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names1) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
    

    def go_to_pose(self, arg_pose):

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # self._group.setPlanningTime(10)
        self._group1.set_pose_target(arg_pose)
        #self._group1.setPlanningTime(10)
        flag_plan = self._group1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group1.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group1.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)
        plan = self._group1.plan()

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

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
    
    def set_joint_angle_1(self, arg_list_joint_angles):

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group2.set_joint_value_target(arg_list_joint_angles)
        self._group2.plan()
        flag_plan = self._group2.go(wait=True)

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)  
    
    def transform_pose(self, src):
        try:
            # self.listener.waitForTransform("ebot_base" , "pepper" , rospy.Time() , rospy.Duration(4.0))
            rospy.loginfo("in the transform function")
            transform = []
            #trans = self.tf_buffer.lookup_transform('ebot_base' , 'fruit_red' , rospy.Time())
            listener = tf.TransformListener()
            listener.waitForTransform("ebot_base", src, rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('ebot_base', src,rospy.Time())


            print("TRANSFORM :" , trans, rot)

            return trans, rot
        except:
            return [],[]

def main():
    rospy.init_node("peppercatcher", anonymous=True)
    try:
        ms = ManiStack()
        flag1 = False 
        attempt = 0 
        attempt2 = 0
        arm_rotation = 0
        flag2 = False
        # ms.pluck_pub.publish(String("False"))
        detect_pose = [math.radians(100),math.radians(-25),math.radians(-54),math.radians(82),math.radians(-7),math.radians(0)]
        inter_pose =  [math.radians(-257),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(-1)]

        inter_pose2 =  [math.radians(-257),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(90)]

        inter_pose_1=  [math.radians(-88),math.radians(-3),math.radians(-37),math.radians(37),math.radians(0),math.radians(-1)]

        inter_pose2_1=  [math.radians(-288),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(90)]

        yellow_drop = [math.radians(9),math.radians(-7),math.radians(3),math.radians(-1),math.radians(-2),math.radians(0)]
        red_drop_1 = [math.radians(-31),math.radians(-7),math.radians(3),math.radians(-1),math.radians(-2),math.radians(0)]
        gripper_pose_open = [math.radians(0)]
        gripper_pose_close = [math.radians(26)]

        if arm_rotation == 0:

            pose = inter_pose_1
        else :

            pose = inter_pose

        ms.set_joint_angles(pose)
        rospy.sleep(2)

        while True:
            ms.pluck_pub.publish(String("False"))

            transform_yellow, rot_yellow=ms.transform_pose("fruit_yellow_1")
            transform_red, rot_red=ms.transform_pose("fruit_red_1")

            if len(transform_red)!=0:

                t2 = geometry_msgs.msg.TransformStamped()
                t2.header.frame_id = "ebot_base"
                t2.header.stamp = rospy.Time.now()
                t2.child_frame_id = "fruit_red"
                t2.transform.translation.x = round(transform_red[0] ,2 ) 
                t2.transform.translation.y = round(transform_red[1] ,2 ) 
                t2.transform.translation.z = round(transform_red[2] ,2 ) 
                t2.transform.rotation.x = 0
                t2.transform.rotation.y = 0
                t2.transform.rotation.z = 0
                t2.transform.rotation.w = 1            
                tfm2 = tf2_msgs.msg.TFMessage([t2])
                ms.pub_tf2.publish(tfm2)

                red_pose_interpose = geometry_msgs.msg.Pose()
                red_pose_interpose.position.x = round(transform_red[0] ,2 ) 
                red_pose_interpose.position.y = round(transform_red[1] ,2 ) - 0.27
                red_pose_interpose.position.z = round(transform_red[2] ,2 ) - 0.01

                red_pose = geometry_msgs.msg.Pose()
                red_pose.position.x = round(transform_red[0] ,2 ) 
                red_pose.position.y = round(transform_red[1] ,2 ) - 0.32
                red_pose.position.z = round(transform_red[2] ,2 ) - 0.01

                while not flag2 and attempt2 < 11 :

                    rospy.loginfo("going to the red pose ")
                    if attempt2 < 6:
                        second_pose = ms.go_to_pose(red_pose_interpose)
                        attempt2 += 1

                    if attempt2 >= 6 and attempt2 < 11 :
                        flag2 = ms.go_to_pose(red_pose)
                        attempt2 += 1 

                ms.set_joint_angle_1(gripper_pose_close)
                ms.set_joint_angles(red_drop_1) 
                ms.set_joint_angle_1(gripper_pose_open)
                arm_rotation = 1  
                ms.pluck_pub.publish("True")

            
            if len(transform_yellow)!=0:
                t3 = geometry_msgs.msg.TransformStamped()
                t3.header.frame_id = "ebot_base"
                t3.header.stamp = rospy.Time.now()
                t3.child_frame_id = "fruit_yellow"
                t3.transform.translation.x = round(transform_yellow[0] ,2 ) 
                t3.transform.translation.y = round(transform_yellow[1] ,2 )
                t3.transform.translation.z = round(transform_yellow[2] ,2 ) 
                t3.transform.rotation.x = 0
                t3.transform.rotation.y = 0
                t3.transform.rotation.z = 0
                t3.transform.rotation.w = 1            
                tfm3 = tf2_msgs.msg.TFMessage([t3])
                ms.pub_tf3.publish(tfm3)

                yellow_pose = geometry_msgs.msg.Pose()
                yellow_pose.position.x = round(transform_yellow[0] ,2 ) 
                yellow_pose.position.y = round(transform_yellow[1] ,2 ) - 0.27
                yellow_pose.position.z = round(transform_yellow[2] ,2 ) - 0.01

                yellow_inter_pose = geometry_msgs.msg.Pose()
                yellow_inter_pose.position.x = round(transform_yellow[0] ,2 ) 
                yellow_inter_pose.position.y = round(transform_yellow[1] ,2 ) - 0.32
                yellow_inter_pose.position.z = round(transform_yellow[2] ,2 ) - 0.01

                print(detect_pose)    

                rospy.loginfo("Trying to go to the pose")



                while not flag1 and attempt < 11 :
                    if attempt < 6:
                        first_pose = ms.go_to_pose(yellow_inter_pose)
                        attempt += 1
                        # rospy.loginfo("Unable to reach")
                    if attempt >= 6 and  attempt < 11:
                        flag1 = ms.go_to_pose(yellow_pose)
                        attempt += 1
                        rospy.loginfo("Reached the Pose")
                        rospy.loginfo(attempt)
                
                ms.set_joint_angle_1(gripper_pose_close)
                ms.set_joint_angles(yellow_drop) 
                ms.set_joint_angle_1(gripper_pose_open)
                arm_rotation = 0
                ms.pluck_pub.publish("True")
                
            
    except Exception as e:
        print("Error:", str(e))    


if __name__=="__main__" :
    main()
    rospy.spin()