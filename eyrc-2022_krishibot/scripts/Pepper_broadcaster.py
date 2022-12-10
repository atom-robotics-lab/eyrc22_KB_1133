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
# Author List:		[ Arjun K Haridas, Bhavay Garg ]
# Filename:			percepStack.py
# Functions:		
# 					[ mask, img_clbk, depth_clbk, image_processing, main ]


####################### IMPORT MODULES #######################
import cv2 
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import imutils
import message_filters
import matplotlib.pyplot as plt
#from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point,Pose
import tf
import os

import subprocess



##############################################################




class PerceptionStack:
    def __init__(self) :
        self.red_mask_lower = (0, 112, 116)
        self.red_mask_upper = (10, 200, 255)

        self.yellow_mask_lower = (10, 160, 150)
        self.yellow_mask_upper = (30, 250, 255)        
        self.bridge = CvBridge()
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf1 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        sub_rgb = message_filters.Subscriber("/camera/color/image_raw2", Image)
        sub_depth = message_filters.Subscriber("/camera/depth/image_raw2", Image)
        self.pub=rospy.Publisher('/pepper', String, queue_size = 1)
        self.yellow_pub=rospy.Publisher('/fruit_yellow', String, queue_size = 1)
        self.red_pub=rospy.Publisher('/fruit_red', String, queue_size = 1)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

        
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None
        self.found=False
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self._planning_group1 = "arm"
        self._planning_group2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)
        self.x = 0
        self.y = 0
        self.z = 0

        #self._group1.setPlannerId("RRTConnectkConfigDefault")

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

    def go_to_pose(self, arg_pose):

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)
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

        #if not plan.joint_trajectory.points :
            #rospy.logerr("Unreachable pose coordinates")

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
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

    def rgb_callback(self, rgb_message) :

        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape
    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    def find_transforms(self,pose, depth_val) : # Finds XYZ coordinates
        transforms = []
        fx, fy = [554.3827128226441, 554.3827128226441]
        cx, cy = [320.5, 240.5]

        #tf = TransformFrames()
        tf_buffer = tf2_ros.Buffer()
        tf_listener=tf2_ros.TransformListener(tf_buffer)

        #pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))

        X , Y , Z = 0 , 0, 0
        for i in range(len(pose)) :
            current_pose, current_depth = pose[i], depth_val[i]
            X = current_depth * ((current_pose[0]-cx)/fx)
            Y = current_depth * ((current_pose[1]-cy)/fy)
            Z = current_depth
            #print(X , Y , Z )
            transforms.append([X,Y,Z])

        return transforms

    def detect(self):
        pose=self.rgb_image_processing()
        self.pose=pose
        #cv2.imshow("depth",self.depth_image)
        #cv2.waitKey(0)
        depth=self.depth_image_processing(pose)
        self.depth=depth
        print("Pose:",pose,type(pose))
        print("Depth:",depth,type(depth))
        self.XYZ=self.find_transforms(pose,depth) # add indexing to depth and remove in case of only one bell peper
        print("Output XYZ:",self.XYZ)
        self.pub.publish(str(self.XYZ))

        if len(self.XYZ)>1:
            self.found=True
        else:
            self.found=False


    def callback(self,depth_data, rgb_data) :
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        if self.found:
            self.red_pub.publish(str(self.XYZ[0]))
            self.yellow_pub.publish(str(self.XYZ[1]))
            
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "camera_depth_frame2"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "fruit_yellow"
            t.transform.translation.x = self.XYZ[0][0]
            t.transform.translation.y = self.XYZ[0][1]
            t.transform.translation.z = self.XYZ[0][2]
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1            
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

            t1 = geometry_msgs.msg.TransformStamped()
            t1.header.frame_id = "camera_depth_frame2"
            t1.header.stamp = rospy.Time.now()
            t1.child_frame_id = "fruit_red"
            t1.transform.translation.x = self.XYZ[1][0]
            t1.transform.translation.y = self.XYZ[1][1]
            t1.transform.translation.z = self.XYZ[1][2]
            t1.transform.rotation.x = 0
            t1.transform.rotation.y = 0
            t1.transform.rotation.z = 0
            t1.transform.rotation.w = 1            
            tfm1 = tf2_msgs.msg.TFMessage([t1])
            self.pub_tf1.publish(tfm1)



    def mask(self, frame, lower, upper):
    
        obj_radius = []    
        obj_center = []

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:

            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                obj_radius.append(radius)
                obj_center.append(list(center[::-1]))
        #cv2.circle(frame,obj_center[0],30,(0,0,255),2)
        #cv2.imshow("Frame",frame)
        #cv2.waitKey(1)
        return [obj_center,obj_radius]


    def rgb_image_processing(self):

        rgb_image = self.rgb_image  
        #cv2.imshow("rgb",rgb_image)
        #cv2.waitKey(0) 
        #print(rgb_image.shape)
        
        red_mask_center, red_mask_radius = self.mask(rgb_image, self.red_mask_lower, self.red_mask_upper)
        yellow_mask_center, yellow_mask_radius = self.mask(rgb_image, self.yellow_mask_lower, self.yellow_mask_upper)

        #print("red: ", red_mask_center)
        #print("Yellow: ",yellow_mask_center)
        pose = red_mask_center + yellow_mask_center    

        for i in range(len(red_mask_center)) :
            cv2.circle(rgb_image, (int(red_mask_center[i][0]), int(red_mask_center[i][1])), int(red_mask_radius[i]),(0, 255, 255), 2)
            cv2.circle(rgb_image, red_mask_center[i], 5, (0, 0, 255), -1)

        for i in range(len(yellow_mask_center)) :
            cv2.circle(rgb_image, (int(yellow_mask_center[i][0]), int(yellow_mask_center[i][1])), int(yellow_mask_radius[i]),(0, 255, 255), 2)
            cv2.circle(rgb_image, yellow_mask_center[i], 5, (0, 0, 255), -1)        
   
        return pose


    def depth_image_processing(self, pose) :

        depth_val = []
        depth_array = np.array(self.depth_image, dtype=np.float32)
        
        #print("POSE : ", pose)
        for i in range(len(pose)):
            #x_center, y_center = int(pose[i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose[i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
            x_center, y_center = int(pose[i][0]), int(pose[i][1])
 
            depth_val.append(depth_array[x_center, y_center])  

        return depth_val

    def transform_pose(self):
        # self.listener.waitForTransform("ebot_base" , "pepper" , rospy.Time() , rospy.Duration(4.0))
        rospy.loginfo("in the transform function")
        transform = []
        #trans = self.tf_buffer.lookup_transform('ebot_base' , 'fruit_red' , rospy.Time())
        listener = tf.TransformListener()
        listener.waitForTransform("ebot_base", "fruit_red", rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('ebot_base', 'fruit_red',rospy.Time())

        '''x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        x1 = trans.transform.rotation.x
        y1 = trans.transform.rotation.y
        z1 = trans.transform.rotation.z
        w = trans.transform.rotation.w'''

        #transform = list(trans) + list(rot)
        #bashCommand_source = "source devel/setup.bash"
        #bashCommand_red = "rosrun tf tf_echo ebot_base fruit_red"
        
        #process = subprocess.Popen(bashCommand_source.split(), stdout = subprocess.PIPE)
        #output, error = process.communicate()

        #process = subprocess.Popen(bashCommand_red.split(), stdout = subprocess.PIPE)
        #output, error = process.communicate()

        #res1 = subprocess.run(bashCommand_source.split())
        #output = subprocess.run(bashCommand_red, capture_output=True)

        #transform = output
        print("TRANSFORM :" , trans, rot)

        return trans, rot



def main():
    rospy.init_node("pepperfinder", anonymous=True)
    try:
        ps = PerceptionStack()
        detect_pose = [math.radians(100),math.radians(-25),math.radians(-54),math.radians(82),math.radians(-7),math.radians(0)]
        inter_pose = [math.radians(-80),math.radians(0),math.radians(0),math.radians(0),math.radians(0),math.radians(0)]
        inter_pose2 =  [math.radians(-257),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(-1)]
        arjun_inter_pose =  [math.radians(-244),math.radians(17),math.radians(1),math.radians(-25),math.radians(0),math.radians(-180)]
        ps.set_joint_angles(inter_pose)
        ps.set_joint_angles(inter_pose2)
        rospy.loginfo("publishing the transform")
        rospy.loginfo("Detecting Peppers")
        ps.detect()
        rospy.sleep(1)
        rospy.loginfo("Got the xyz values now finding the transforms")
        transform,rot=ps.transform_pose()
        rospy.loginfo("Got the transforms")
        rospy.loginfo("Trying to go to the pose")
        print(transform)
        ps.go_to_pose(transform)
        rospy.loginfo("Reached the Pose")

        
        
        #print(transform)

        # inter_pose = [math.radians(-80),math.radians(0),math.radians(0),math.radians(0),math.radians(0),math.radians(0)]
        # inter_pose2 =  [math.radians(-244),math.radians(17),math.radians(1),math.radians(-25),math.radians(0),math.radians(-180)]
        # yellow_basket =  [math.radians(11),math.radians(0),math.radians(0),math.radians(0),math.radians(0),math.radians(0)]

        # detect_pose = geometry_msgs.msg.Pose()
        # detect_pose.position.x = 0.1096910324768404
        # detect_pose.position.y = -0.343399873902921
        # detect_pose.position.z =  1.0550548568280893

        # detect_pose.orientation.x = -0.14770761462167648
        # detect_pose.orientation.y = 0.9886845985686313
        # detect_pose.orientation.z = -0.026166747456944396
        # detect_pose.orientation.w = 0.000725578033768894

        # rospy.sleep(10)
        
        # flag = False
        # while not flag:  
        #     trans, rot = ps.transform_pose()

        #     ps.set_joint_angles(inter_pose2)

        #     yellow_pose = geometry_msgs.msg.Pose()
        #     yellow_pose.position.x = trans[0]
        #     yellow_pose.position.y = trans[1]
        #     yellow_pose.position.z =  trans[2]

        #     yellow_pose.orientation.x = -0.11339548561710626
        #     yellow_pose.orientation.y = 0.9931652834334606
        #     yellow_pose.orientation.z = -0.025094005244503455
        #     yellow_pose.orientation.w = 0.011596315146774774       
            
        #     print("YELLOW POSE")
        #     flag = ps.go_to_pose(yellow_pose)
        '''yellow_pose, red_pose = ps.rgb_image_processing()
            #print("red_pose : ", red_pose)
            #print("yellow_pose : ", yellow_pose)
            depth_yellow, depth_red = ps.depth_image_processing(yellow_pose), ps.depth_image_processing(red_pose)

            yellow_transforms, red_transforms = find_transforms(yellow_pose, depth_yellow), find_transforms(red_pose, depth_red)

            print("YELLOW TRANSFORM : ", yellow_transforms)
            print("RED TRANSFORM : ", red_transforms)

            for i in range(len(yellow_transforms)) :
                #ur5.set_joint_angles(inter_pose2)

                ur5.go_to_predefined_pose("open", 2)


                flag = ur5.go_to_pose(yellow_transforms[i])

                #print(transforms[i])
                #transforms[i].pose.position.y -= 0.15
                #ur5.go_to_pose(transforms[i])
                #flag = False

                #while not flag : 
                ur5.go_to_predefined_pose("close", 2)

                ur5.set_joint_angles(yellow_basket)

                ur5.go_to_predefined_pose("open", 2)

            for i in range(len(red_transforms)) :
                #ur5.set_joint_angles(inter_pose2)

                ur5.go_to_predefined_pose("open", 2)


                flag = ur5.go_to_pose(red_transforms[i])

                #print(transforms[i])
                #transforms[i].pose.position.y -= 0.15
                #ur5.go_to_pose(transforms[i])
                #flag = False

                #while not flag : 
                ur5.go_to_predefined_pose("close", 2)

                ur5.set_joint_angles(yellow_basket)

                ur5.go_to_predefined_pose("open", 2)'''

    except Exception as e:
        print("Error:", str(e))    

if __name__=="__main__" :
    main()
    rospy.spin()