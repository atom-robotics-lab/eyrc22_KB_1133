#! /usr/bin/env python3

## import the necessary libraries
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

# Initialize class objects
bridge = CvBridge()

# Initialize Global variables

# Write your own custom functions



# Call Back functions
def img_callbck(img_msg):
    '''
    Call back function for image
    Convert the image in a cv2 format and then 
    pass it to image_processsing function
    '''
    global pub #, add global variable if any

    # Your code here to convert the image to cv2 format
    # and save the converted in image in `image` variable.

    pose = image_processing(image)
    pub_rgb.publish(str(pose))

def depth_callbck(depth_msg)
    '''
    Find the depth of the centroid from the image_processing, 
    i.e. finding the depth value of the particular 
    pixel via the image.

    HINT: the shape of both depth and rgb is different 
    '''

    depth_val = # value of depth as per the centroid of the rgb picture after remapping image
    pub_depth.publish(str(depth_val))

# --------------------------------------------------------------------------------------------#
# Do not modify the below function name and return 
# Only do the changes in the specified portion for this
# function.

def image_processing(image):
    '''
    1. Convert the image format, to see you can use cv2.imshow() 
    2. Use moments to find centroid of the image.
    3. Set the x and y value for return as the centroid value of the detected fruit
       and append in the pose variable by doing `pose.append(x, y)`.
    4. If multiple fuits found then do append multiple times.
    '''
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################

    ##########################################################################################
    return pose
#----------------------------------------------------------------------------------------------#

rospy.init_node("percepStack", anonymous=True)
# Only for one image, you have to iterate over three images in the same script
# and pulish the centroid and depth in the same script for three images
sub_image_color_1 = rospy.Subscriber("/device_0/sensor_1/Color_0/image/data_1", Image, img_callbck)
sub_image_depth_1 = rospy.Subscriber("/device_0/sensor_0/Depth_0/image/data_1", Image, depth_callbck)

pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)