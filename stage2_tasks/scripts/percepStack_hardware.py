#! /usr/bin/env python3

# Team ID:			[ 1133 ]
# Author List:		[ Arjun K Haridas, Bhavay Garg , Ayan Goel , Divyansh Sharma ]
# Filename:			percepStack.py
# Theme:            Krishibot 
# Functions:		[ mask, img_clbk, depth_clbk, image_processing, main , rgb_processing, find_transpose, detect]
# Global Variables  rgb_shape, depth_shape 

####################### IMPORT MODULES #######################
import cv2 
import rospy
import imutils
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import tf2_ros
import tf2_msgs.msg
import tf
import math
##############################################################


class PercepStack():

    # Function name: init 
    # input: none 
    # output: none 
    # Description: Initialises standard variables and cteates the subscribers and publishers

    def __init__(self) -> None:

        self.red_mask_lower = (150, 109, 51)
        self.red_mask_upper = (179, 255, 255)
        
        # Values to be used only in Evening
        # self.yellow_mask_lower = (2, 91, 105)
        # self.yellow_mask_upper = (27, 255, 255)  
        
        # self.yellow_mask_lower = (2, 125, 132)
        # self.yellow_mask_upper = (27, 255, 255)  

        self.yellow_mask_lower = (10, 134, 146)
        self.yellow_mask_upper = (27, 255, 255) 

        self.bridge = CvBridge()

        sub_rgb = message_filters.Subscriber("/camera/color/image_raw", Image)
        # sub_rgb = message_filters.Subscriber("/camera/color/image_raw/compressed_throttle", CompressedImage)
        sub_depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf1 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf2 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_tf3 = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub=rospy.Publisher('/pepper', String, queue_size = 1)
        self.yellow_pub=rospy.Publisher('/fruit_yellow', String, queue_size = 1)
        self.red_pub=rospy.Publisher('/fruit_red', String, queue_size = 1)
        self.found_pub=rospy.Publisher('/found', String, queue_size = 1)
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None
        self.found=False
    
    # Function Name: rgb_callback 
    # Input: rgb_message
    # Output: None 
    # Description: Decodes the incoming rgb_message and saves it in rgb_image variable

    def rgb_callback(self, rgb_message) :
        try :
            np_arr = np.frombuffer(rgb_message.data, np.uint8)
            # self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
            
            self.rgb_shape = self.rgb_image.shape

        except Exception as e:
            print("rgb_callback exception : ", e)
            pass
    
    
    # Function Name: depth_callback 
    # Input: depth_message 
    # Output: None
    # Description: Decodes the depth image and saves it to variable depth_image 
    
    def depth_callback(self, depth_message) :
        self.depth_image = snelf.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    def handle_turtle_pose(self, x , y, z , pepper ) :
        br = tf.TransformBroadcaster()
        #time_1 = rospy.Time.now() + 0.5

        br.sendTransform((x , y , z),
                        (0,0,0,1),
                        rospy.Time.now(),
                        pepper,
                        "camera_link")
        print(x ,y ,z )                
        print("Transform broadcast")


    # Function Name: find_transforms 
    # Input: poses and their depths
    # Output: tf of fruits 
    # Description: Maps the given poses and depths to real world cordinates

    def find_transforms(self,pose, depth_val) : # Finds XYZ coordinates
        transforms = {"red":[],"yellow":[]}

        fx, fy = [554.387, 554.387]
        cx, cy = [320.5, 240.5]

        #tf = TransformFrames()
        tf_buffer = tf2_ros.Buffer()
        tf_listener=tf2_ros.TransformListener(tf_buffer)

        #pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))

        X , Y , Z = 0 , 0, 0
        for i in range(len(pose["red"])) :
            current_pose, current_depth = pose["red"][i], depth_val["red"][i]
            Y = -1*current_depth * ((current_pose[1]-cx)/fx)* 0.001
            Z = -1*current_depth * ((current_pose[0]-cy)/fy)* 0.001
            X = current_depth * 0.001
            # print(X , Y , Z )
            transforms["red"].append([X,Y,Z])
        
        X , Y , Z = 0 , 0, 0
        for i in range(len(pose["yellow"])) :
            current_pose, current_depth = pose["yellow"][i], depth_val["yellow"][i]
            Y = -1*current_depth * ((current_pose[1]-cx)/fx) * 0.001
            Z = -1*current_depth * ((current_pose[0]-cy)/fy) * 0.001
            X = current_depth * 0.001
            # print(X , Y , Z )
            transforms["yellow"].append([X,Y,Z])

        print("\nTRANSFORMS : \n")
        rospy.loginfo(transforms)
        return transforms


    # Function Name: detect 
    # Input: None
    # Output: None 
    # Description: Evaluates the incoming images and checks if a fruit is detected

    def detect(self):
        pose=self.rgb_image_processing()
        self.pose=pose
        #cv2.imshow("depth",self.depth_image)
        #cv2.waitKey(0)
        depth=self.depth_image_processing(pose)
        self.depth=depth
        #print("Pose:",pose,type(pose))
        #print("Depth:",depth,type(depth))
        self.XYZ=self.find_transforms(pose,depth) # add indexing to depth and remove in case of only one bell peper
        ##print("Output XYZ:",self.XYZ)
        self.pub.publish(str(self.XYZ))

        l=len(self.XYZ["red"])+len(self.XYZ["yellow"])
        if l>=1:
            self.found=True
            self.found_pub.publish("Stop")
            rospy.loginfo("STOP")
            self.child_id_red = "fruit_red_1"
            self.child_id_yellow= "fruit_yellow_1"

        else:
            print("detect function : length is less than 1")
            self.found_pub.publish("Move")
            rospy.loginfo("Move")
            self.found=False


    # Function Name: callback 
    # Input: depth_data,rgb_data
    # Output: None 
    # Description: Takes in the incoming rgb and depth data and calls respective functions

    def callback(self,depth_data, rgb_data) :

        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        # self.handle_turtle_pose()
        self.red_peper = "Red_pepper"
        self.yellow_pepper = "Yellow_pepper"

        if self.found:
            #print("Callback : Pepper Found ")
            self.red_pub.publish(str(self.XYZ["red"]))
            self.yellow_pub.publish(str(self.XYZ["yellow"]))

            for i in range(len(self.XYZ["red"])):
                #print("\nSend Red Transform : \n")
                self.handle_turtle_pose(self.XYZ["red"][i][0],self.XYZ["red"][i][1],self.XYZ["red"][i][2] , self.red_peper)
            
            for i in range(len(self.XYZ["yellow"])):
                #print("\nSend Yellow Transform : \n")
                self.handle_turtle_pose(self.XYZ["yellow"][i][0],self.XYZ["yellow"][i][1],self.XYZ["yellow"][i][2] , self.yellow_pepper)


    # def transform_pose(self, src):
    #     try:
    #         # self.listener.waitForTransform("ebot_base" , "pepper" , rospy.Time() , rospy.Duration(4.0))
    #         #rospy.loginfo("in the transform function")
    #         transform = []
    #         #trans = self.tf_buffer.lookup_transform('ebot_base' , 'fruit_red' , rospy.Time())
    #         listener = tf.TransformListener()
    #         listener.waitForTransform("ebot_base", src, rospy.Time(), rospy.Duration(4.0))
    #         (trans, rot) = listener.lookupTransform('ebot_base', src,rospy.Time())


    #         #print("TRANSFORM :" , trans, rot)

    #         return trans, rot
    #     except:
    #         return [],[]
    
    # def ebot_base_transform(self):

    #     transform_yellow, rot_yellow= self.transform_pose("fruit_yellow_1")
    #     transform_red, rot_red= self.transform_pose("fruit_red_1")

    #     if len(transform_red)!=0:
    #         # attempt2 = 0
    #         t2 = geometry_msgs.msg.TransformStamped()
    #         t2.header.frame_id = "ebot_base"
    #         t2.header.stamp = rospy.Time.now()
    #         t2.child_frame_id = "fruit_red"
    #         t2.transform.translation.x = round(transform_red[0] ,2 ) 
    #         t2.transform.translation.y = round(transform_red[1] ,2 ) 
    #         t2.transform.translation.z = round(transform_red[2] ,2 ) 
    #         t2.transform.rotation.x = 0
    #         t2.transform.rotation.y = 0
    #         t2.transform.rotation.z = 0
    #         t2.transform.rotation.w = 1            
    #         tfm2 = tf2_msgs.msg.TFMessage([t2])
    #         self.pub_tf2.publish(tfm2)

    #     if len(transform_yellow)!=0:
    #         t3 = geometry_msgs.msg.TransformStamped()
    #         t3.header.frame_id = "ebot_base"
    #         t3.header.stamp = rospy.Time.now()
    #         t3.child_frame_id = "fruit_yellow"
    #         t3.transform.translation.x = round(transform_yellow[0] ,2 ) 
    #         t3.transform.translation.y = round(transform_yellow[1] ,2 )
    #         t3.transform.translation.z = round(transform_yellow[2] ,2 ) 
    #         t3.transform.rotation.x = 0
    #         t3.transform.rotation.y = 0
    #         t3.transform.rotation.z = 0
    #         t3.transform.rotation.w = 1            
    #         tfm3 = tf2_msgs.msg.TFMessage([t3])
    #         self.pub_tf3.publish(tfm3)


    # Function Name: mask 
    # Input: frame,lower,upper
    # Output: [obj_center,obj_radius] 
    # Description: processes the image using the given hsv values and gives the centers and radii of detected fruits

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
                cv2.circle(frame,obj_center[0][::-1],30,(255,0,0),4)
                cv2.imshow("Frame",frame)
                cv2.waitKey(1)
        return [obj_center,obj_radius]

    # Function Name: rgb_image_processing 
    # Input: None
    # Output: Pose 
    # Description: Processes the rgb image to find the yellow and red fruits

    def rgb_image_processing(self):

        try  :
            rgb_image = self.rgb_image  
            
            cv2.imshow("rgb",rgb_image)
            cv2.waitKey(1) 
            # print(rgb_image.shape)

            red_mask_center, red_mask_radius = self.mask(rgb_image, self.red_mask_lower, self.red_mask_upper)
            yellow_mask_center, yellow_mask_radius = self.mask(rgb_image, self.yellow_mask_lower, self.yellow_mask_upper)
            pose={}
            pose["red"]=red_mask_center
            pose["yellow"]=yellow_mask_center
            print("red: ", pose["red"])
            print("Yellow: ",pose["yellow"])    
            return pose

        except Exception as e:
            print("rgb_image_processing exception : ", e)
            pose={"red":[],"yellow":[]}
            return pose

    # Function Name: depth_image_processing 
    # Input: pose
    # Output: depth array 
    # Description: Processes the depth image using the given pose and finds the depth of the fruits

    def depth_image_processing(self, pose) :

        depth_val = {"red":[],"yellow":[]}
        depth_array = np.array(self.depth_image, dtype=np.float32)
        
        # print("POSE : ", pose)
        invalid=[]
        for i in range(len(pose["red"])):
            x_center, y_center = int(pose["red"][i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose["red"][i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
            # x_center, y_center = int(pose["red"][i][0]), int(pose["red"][i][1])
            print(depth_array[x_center, y_center])
            

            # depth_val["red"].append(depth_array[x_center, y_center])  
            if depth_array[x_center, y_center] <=1500 :
                depth_val["red"].append(depth_array[x_center, y_center])  
            else:
                invalid.append(pose["red"][i])
                print("Depth filter removed a red pose")
        for i in invalid:
            pose["red"].remove(i)
        
        invalid=[]
        for i in range(len(pose["yellow"])):
            x_center, y_center = int(pose["yellow"][i][0]*(self.depth_shape[0]/self.rgb_shape[0])), int(pose["yellow"][i][1]*(self.depth_shape[1]/self.rgb_shape[1]))
            # x_center, y_center = int(pose["yellow"][i][0]), int(pose["yellow"][i][1])
            print(depth_array[x_center, y_center])
            
            
            # depth_val["yellow"].append(depth_array[x_center, y_center]) 
            if depth_array[x_center, y_center] <=1500 :
                depth_val["yellow"].append(depth_array[x_center, y_center]) 
            else:
                invalid.append(pose["yellow"][i])
                print("Depth filter removed a yellow pose")
        for i in invalid:
            pose["yellow"].remove(i)

        return depth_val

    # def set_joint_angles(self, arg_list_joint_angles):

    #     list_joint_values = self._group1.get_current_joint_values()
    #     rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
    #     rospy.loginfo(list_joint_values)

    #     self._group1.set_joint_value_target(arg_list_joint_angles)
    #     self._group1.plan()
    #     flag_plan = self._group1.go(wait=True)

    #     list_joint_values = self._group1.get_current_joint_values()
    #     rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
    #     rospy.loginfo(list_joint_values)

    #     pose_values = self._group1.get_current_pose().pose
    #     rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
    #     rospy.loginfo(pose_values)   



# Function Name: main 
# Input: None
# Output: None 
# Description: Inits the node and continously calls the detect function

def main():
    rospy.init_node("pepperfinder", anonymous=True)
    # try:
    ps = PercepStack()
    rospy.sleep(1)
    inter_pose =  [math.radians(-257),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(-1)]
    inter_pose_1=  [math.radians(-88),math.radians(-3),math.radians(-37),math.radians(37),math.radians(0),math.radians(-1)]

    while True:
        ps.detect()
        
    # except Exception as e:
        # print("Error:", str(e))    


if __name__=="__main__" :
    main()
    rospy.spin()