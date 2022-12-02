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
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import imutils
import message_filters
import matplotlib.pyplot as plt




##############################################################

class PerceptionStack:
    def __init__(self) :
        self.red_mask_lower = (148, 112, 116)
        self.red_mask_upper = (179, 255, 255)

        self.yellow_mask_lower = (10, 133, 98)
        self.yellow_mask_upper = (26, 255, 255)        
        self.bridge = CvBridge()

        sub_rgb = rospy.Subscriber("/camera/color/image_raw2", Image, self.rgb_callback)
        sub_depth = rospy.Subscriber("/camera/depth/image_raw2", Image, self.depth_callback)

        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None

    def rgb_callback(self, rgb_message) :

        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape
    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

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
                obj_center.append(list(center))



        return [obj_center,obj_radius]


    def rgb_image_processing(self):

        rgb_image = self.rgb_image   
        
        red_mask_center, red_mask_radius = self.mask(rgb_image, self.red_mask_lower, self.red_mask_upper)
        yellow_mask_center, yellow_mask_radius = self.mask(rgb_image, self.yellow_mask_lower, self.yellow_mask_upper)

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
        
        
        for i in range(len(pose)):
            x_center, y_center = int(pose[i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose[i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
                   
            depth_val.append(round(depth_array[x_center, y_center]/1000,1))  

        return depth_val


if __name__=="__main__" :
    ps = PerceptionStack()

    while True :
        img = cv2.imread("sample.png")

        pose, output = ps.rgb_image_processing(img)
        print("pose : {}".format(pose))
        cv2.imshow("Output", output)

        if cv2.waitKey(1) and 0xFF == ord('q') :
            break
        
        



