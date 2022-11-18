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

# You can add more if required
##############################################################


# Initialize Global variables
red_mask_lower = (148, 112, 116)
red_mask_upper = (179, 255, 255)

yellow_mask_lower = (10, 133, 98)
yellow_mask_upper = (26, 255, 255)

pub_rgb = rospy.Publisher('/center_rgb', String, queue_size=20)
pub_depth = rospy.Publisher('/center_depth', String, queue_size=20)

pose1 = []
pose2 = []
pose3 = []

rgb_shape, depth_shape = None, None

################# ADD UTILITY FUNCTIONS HERE #################

def mask(frame, lower, upper):
    
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

##############################################################

def callback1(depth_data, rgb_data) :

    depth_clbck1(depth_data)
    img_clbck1(rgb_data)
    

    #pub_rgb.publish(str(pose_result))
    #pub_depth.publish(str(depth_result))

    #print("pose", pose_result)
    #print("depth", depth_result)
    #print()
    # 
    
def callback2(depth_data, rgb_data) :

    depth_clbck2(depth_data)
    img_clbck2(rgb_data)
    

    #pub_rgb.publish(str(pose_result))
    #pub_depth.publish(str(depth_result))

    #print("pose", pose_result)
    #print("depth", depth_result)
    #print()

def callback3(depth_data, rgb_data) :

    depth_clbck3(depth_data)
    img_clbck3(rgb_data)
    

    #pub_rgb.publish(str(pose_result))
    #pub_depth.publish(str(depth_result))

    #print("pose", pose_result)
    #print("depth", depth_result)
    #print()



def img_clbck1(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    
          
    global pub_rgb, pose1, rgb_shape #, add global variable if any

    ############################### Add your code here #######################################
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    image = rgb_image
    rgb_shape = rgb_image.shape
    print("rgb : ", rgb_shape)
    
    ##########################################################################################
    pose1 = image_processing(image)   
    pub_rgb.publish(str(pose1))

def img_clbck2(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    
          
    global pub_rgb,pose2, rgb_shape #, add global variable if any

    ############################### Add your code here #######################################
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    image = rgb_image
    rgb_shape = rgb_image.shape
    print("rgb : ", rgb_shape)
    
    ##########################################################################################
    pose2 = image_processing(image)   
    pub_rgb.publish(str(pose2))

def img_clbck3(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''
    
          
    global pub_rgb, pose3, rgb_shape #, add global variable if any

    ############################### Add your code here #######################################
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    image = rgb_image
    rgb_shape = rgb_image.shape
    print("rgb : ", rgb_shape)
    
    ##########################################################################################
    pose3 = image_processing(image)   
    pub_rgb.publish(str(pose3))

def depth_clbck1(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################
    
    global pose1, depth_shape

    bridge = CvBridge()
    
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
    depth_shape = depth_image.shape    

    print("depth : ", depth_shape)    
    depth_array = np.array(depth_image, dtype=np.float32)

    #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    
    depth_8 = (depth_array * 255).round().astype(np.uint8)

    

    for i in range(len(pose1)):
        x_center, y_center = int(pose1[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose1[i][1]*(depth_shape[1]/rgb_shape[1]))
        print(x_center, y_center)
        cv2.circle(depth_image, (x_center, y_center), 25, 0, -1)
        cv2.circle(depth_image, (x_center, y_center), 10, 1, -1)
        
        depth_val.append(round(depth_array[x_center, y_center]/1000,1))        
        #depth_val.append(depth_image[y_center, x_center])

    #for i in range(len(pose)) :
        #cv2.circle(depth_image, (int(pose[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose[i][1]*(depth_shape[1]/rgb_shape[1]))), 2,(255, 255, 255), -1)
        #cv2.circle(depth_image, yellow_mask_center[i], 5, (0, 0, 255), -1)

    #cv2.imshow("depth_image", depth_image)
    # if cv2.waitKey(30) & 0xFF == ord('q'):
    #     exit()
    

    
    print("pose : ", pose1)
    print("depth : ", depth_val)

    print()
    
    ##########################################################################################
    #return depth_val
    pub_depth.publish(str(depth_val))

def depth_clbck2(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################
    
    global pose2, depth_shape

    bridge = CvBridge()
    
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
    depth_shape = depth_image.shape    

    print("depth : ", depth_shape)    
    depth_array = np.array(depth_image, dtype=np.float32)

    #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    
    depth_8 = (depth_array * 255).round().astype(np.uint8)

    

    for i in range(len(pose2)):
        x_center, y_center = int(pose2[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose2[i][1]*(depth_shape[1]/rgb_shape[1]))
        print(x_center, y_center)
        cv2.circle(depth_image, (x_center, y_center), 25, 0, -1)
        cv2.circle(depth_image, (x_center, y_center), 10, 1, -1)
        
        depth_val.append(round(depth_array[x_center, y_center]/1000,1))        
        #depth_val.append(depth_image[y_center, x_center])

    #for i in range(len(pose)) :
        #cv2.circle(depth_image, (int(pose[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose[i][1]*(depth_shape[1]/rgb_shape[1]))), 2,(255, 255, 255), -1)
        #cv2.circle(depth_image, yellow_mask_center[i], 5, (0, 0, 255), -1)

    #cv2.imshow("depth_image", depth_image)
    # if cv2.waitKey(30) & 0xFF == ord('q'):
    #     exit()
    

    
    print("pose : ", pose2)
    print("depth : ", depth_val)

    print()
    
    ##########################################################################################
    #return depth_val
    pub_depth.publish(str(depth_val))

def depth_clbck3(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    depth_val = []
    ############################### Add your code here #######################################
    
    global pose3, depth_shape

    bridge = CvBridge()
    
    depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
    depth_shape = depth_image.shape    

    print("depth : ", depth_shape)    
    depth_array = np.array(depth_image, dtype=np.float32)

    #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
    
    depth_8 = (depth_array * 255).round().astype(np.uint8)

    

    for i in range(len(pose3)):
        x_center, y_center = int(pose3[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose3[i][1]*(depth_shape[1]/rgb_shape[1]))
        print(x_center, y_center)
        cv2.circle(depth_image, (x_center, y_center), 25, 0, -1)
        cv2.circle(depth_image, (x_center, y_center), 10, 1, -1)
        
        depth_val.append(round(depth_array[x_center, y_center]/1000,1))        
        #depth_val.append(depth_image[y_center, x_center])

    #for i in range(len(pose)) :
        #cv2.circle(depth_image, (int(pose[i][0]*(depth_shape[0]/rgb_shape[0])), int(pose[i][1]*(depth_shape[1]/rgb_shape[1]))), 2,(255, 255, 255), -1)
        #cv2.circle(depth_image, yellow_mask_center[i], 5, (0, 0, 255), -1)

    #cv2.imshow("depth_image", depth_image)
    # if cv2.waitKey(30) & 0xFF == ord('q'):
    #     exit()
    

    
    print("pose : ", pose3)
    print("depth : ", depth_val)

    print()
    
    ##########################################################################################
    #return depth_val
    pub_depth.publish(str(depth_val))


def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################

    red_mask_center, red_mask_radius = mask(image, red_mask_lower, red_mask_upper)
    yellow_mask_center, yellow_mask_radius = mask(image, yellow_mask_lower, yellow_mask_upper)

    pose = red_mask_center + yellow_mask_center
    

    for i in range(len(red_mask_center)) :
        cv2.circle(image, (int(red_mask_center[i][0]), int(red_mask_center[i][1])), int(red_mask_radius[i]),(0, 255, 255), 2)
        cv2.circle(image, red_mask_center[i], 5, (0, 0, 255), -1)

    for i in range(len(yellow_mask_center)) :
        cv2.circle(image, (int(yellow_mask_center[i][0]), int(yellow_mask_center[i][1])), int(yellow_mask_radius[i]),(0, 255, 255), 2)
        cv2.circle(image, yellow_mask_center[i], 5, (0, 0, 255), -1)
    
    #cv2.imshow("Image", image)
    # if cv2.waitKey(30) & 0xFF == ord('q'):
    #     exit()
    
    ##########################################################################################
    return pose



def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####

    rospy.init_node("percepStack", anonymous=True)

    
    

    # for i in range(3) :

        #print("loop")
        #i = 2

    sub_image_depth_1 = message_filters.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image)
    sub_image_color_1 = message_filters.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image)
    sub_image_depth_2 = message_filters.Subscriber("/device_0/sensor_0/Depth_0/image/data_2", Image)
    sub_image_color_2 = message_filters.Subscriber("/device_0/sensor_1/Color_0/image/data_2", Image)
    sub_image_depth_3 = message_filters.Subscriber("/device_0/sensor_0/Depth_0/image/data_3", Image)
    sub_image_color_3 = message_filters.Subscriber("/device_0/sensor_1/Color_0/image/data_3", Image)

    ts1 = message_filters.ApproximateTimeSynchronizer([sub_image_depth_1, sub_image_color_1], queue_size=10, slop=0.5)
    ts1.registerCallback(callback1)
    ts2 = message_filters.ApproximateTimeSynchronizer([sub_image_depth_2, sub_image_color_2], queue_size=10, slop=0.5)
    ts2.registerCallback(callback2)
    ts3 = message_filters.ApproximateTimeSynchronizer([sub_image_depth_3, sub_image_color_3], queue_size=10, slop=0.5)
    ts3.registerCallback(callback3)

    #pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
    #pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)

    ####################################################################################################
    rospy.spin()


if __name__ == '__main__':
    try:
        while True :
            main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")