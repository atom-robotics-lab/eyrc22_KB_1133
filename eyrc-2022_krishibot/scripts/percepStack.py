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
# Functions:		
# 					[ mask, img_clbk, depth_clbk, image_processing, main ]

####################### IMPORT MODULES #######################
import cv2 
import rospy
import imutils
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros
import tf2_msgs.msg

##############################################################


class PercepStack():
    def __init__(self) -> None:
        self.red_mask_lower = (0, 112, 116)
        self.red_mask_upper = (10, 200, 255)

        self.yellow_mask_lower = (10, 160, 150)
        self.yellow_mask_upper = (30, 250, 255)        
        self.bridge = CvBridge()

        sub_rgb = message_filters.Subscriber("/camera/color/image_raw2", Image)
        sub_depth = message_filters.Subscriber("/camera/depth/image_raw2", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf1 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub=rospy.Publisher('/pepper', String, queue_size = 1)
        self.yellow_pub=rospy.Publisher('/fruit_yellow', String, queue_size = 1)
        self.red_pub=rospy.Publisher('/fruit_red', String, queue_size = 1)
        self.found_pub=rospy.Publisher('/found', String, queue_size = 1)
        
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None
        self.found=False
        
    def rgb_callback(self, rgb_message) :

        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape

    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    def find_transforms(self,pose, depth_val) : # Finds XYZ coordinates
        transforms = {"red":[],"yellow":[]}

        fx, fy = [554.3827128226441, 554.3827128226441]
        cx, cy = [320.5, 240.5]

        #tf = TransformFrames()
        tf_buffer = tf2_ros.Buffer()
        tf_listener=tf2_ros.TransformListener(tf_buffer)

        #pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))

        X , Y , Z = 0 , 0, 0
        for i in range(len(pose["red"])) :
            current_pose, current_depth = pose["red"][i], depth_val["red"][i]
            X = current_depth * ((current_pose[0]-cx)/fx)
            Y = current_depth * ((current_pose[1]-cy)/fy)
            Z = current_depth
            #print(X , Y , Z )
            transforms["red"].append([X,Y,Z])
        
        X , Y , Z = 0 , 0, 0
        for i in range(len(pose["yellow"])) :
            current_pose, current_depth = pose["yellow"][i], depth_val["yellow"][i]
            X = current_depth * ((current_pose[0]-cx)/fx)
            Y = current_depth * ((current_pose[1]-cy)/fy)
            Z = current_depth
            #print(X , Y , Z )
            transforms["yellow"].append([X,Y,Z])

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

        l=len(self.XYZ["red"])+len(self.XYZ["yellow"])
        if l>=1:
            self.found=True
            self.found_pub.publish("Stop")
        else:
            self.found=False


    def callback(self,depth_data, rgb_data) :
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        if self.found:
            self.red_pub.publish(str(self.XYZ["red"]))
            self.yellow_pub.publish(str(self.XYZ["yellow"]))

            for i in range(len(self.XYZ["red"])):
                t1 = geometry_msgs.msg.TransformStamped()
                t1.header.frame_id = "camera_depth_frame2"
                t1.header.stamp = rospy.Time.now()
                t1.child_frame_id = "fruit_red_1"
                t1.transform.translation.x = self.XYZ["red"][i][0]
                t1.transform.translation.y = self.XYZ["red"][i][1]
                t1.transform.translation.z = self.XYZ["red"][i][2]
                t1.transform.rotation.x = 0
                t1.transform.rotation.y = 0
                t1.transform.rotation.z = 0
                t1.transform.rotation.w = 1            
                tfm1 = tf2_msgs.msg.TFMessage([t1])
                self.pub_tf1.publish(tfm1)
            
            
            for i in range(len(self.XYZ["yellow"])):
            
                t = geometry_msgs.msg.TransformStamped()
                t.header.frame_id = "camera_depth_frame2"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "fruit_yellow_1"
                t.transform.translation.x = self.XYZ["yellow"][i][0]
                t.transform.translation.y = self.XYZ["yellow"][i][1]
                t.transform.translation.z = self.XYZ["yellow"][i][2]
                t.transform.rotation.x = 0
                t.transform.rotation.y = 0
                t.transform.rotation.z = 0
                t.transform.rotation.w = 1            
                tfm = tf2_msgs.msg.TFMessage([t])
                self.pub_tf.publish(tfm)

            

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
        pose={}
        pose["red"]=red_mask_center
        pose["yellow"]=yellow_mask_center
        #print("red: ", red_mask_center)
        #print("Yellow: ",yellow_mask_center)
        #pose = red_mask_center + yellow_mask_center    

        for i in range(len(red_mask_center)) :
            cv2.circle(rgb_image, (int(red_mask_center[i][0]), int(red_mask_center[i][1])), int(red_mask_radius[i]),(0, 255, 255), 2)
            cv2.circle(rgb_image, red_mask_center[i], 5, (0, 0, 255), -1)

        for i in range(len(yellow_mask_center)) :
            cv2.circle(rgb_image, (int(yellow_mask_center[i][0]), int(yellow_mask_center[i][1])), int(yellow_mask_radius[i]),(0, 255, 255), 2)
            cv2.circle(rgb_image, yellow_mask_center[i], 5, (0, 0, 255), -1)        
   
        return pose


    def depth_image_processing(self, pose) :

        depth_val = {"red":[],"yellow":[]}
        depth_array = np.array(self.depth_image, dtype=np.float32)
        
        #print("POSE : ", pose)
        for i in range(len(pose["red"])):
            #x_center, y_center = int(pose[i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose[i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
            x_center, y_center = int(pose["red"][i][0]), int(pose["red"][i][1])
 
            depth_val["red"].append(depth_array[x_center, y_center])  
        
        
        for i in range(len(pose["yellow"])):
            #x_center, y_center = int(pose[i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose[i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
            x_center, y_center = int(pose["yellow"][i][0]), int(pose["yellow"][i][1])
 
            depth_val["yellow"].append(depth_array[x_center, y_center])  

        return depth_val



def main():
    rospy.init_node("pepperfinder", anonymous=True)
    # try:
    ps = PercepStack()
    rospy.sleep(1)
    while True:
        ps.detect()
        
    # except Exception as e:
        # print("Error:", str(e))    


if __name__=="__main__" :
    main()
    rospy.spin()
