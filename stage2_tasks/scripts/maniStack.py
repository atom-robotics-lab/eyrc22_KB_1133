#! /usr/bin/env python3

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import sys,math
import tf
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Float32MultiArray, String, Empty
from ur_msgs.srv import SetIO
from geometry_msgs.msg import Pose



class Ur5Moveit:

    def __init__(self):


        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


        self.ack_val = 0                # keeping it as default state in zero, as it acknowledges the topic connection
        self.ack_val_poseJoint = 0      # for pose joint val ack
        self._eef_link = "wrist_3_link" # declaring end effector link for the clas manually
        
        # Subscriber nodes
        rospy.Subscriber("/joint_pose_ack", Bool, self.ack_clbck, queue_size=1)
        rospy.Subscriber("/joint_ee_pose", String, self.pose_joint_clbck, queue_size=1)

        # acknowledgment log on terminal
        rospy.logwarn("Make sure that the Manipulation Help node initialization is complete.")
        rospy.loginfo("!!!!!!!!!!!!!!!! UR5_Moveit Initialization complete !!!!!!!!!!!!!!!!!")

        # Publisher variable for joint angles
        self.pub_joint_angles = rospy.Publisher("/set_joint_value_target_wait_topic", 
                                        Float32MultiArray, queue_size=10)

        # Publisher variable for pose values
        self.pub_pose_values = rospy.Publisher("/set_pose_value_target_wait_topic", 
                                        Pose, queue_size=10)
        
        # Publisher variable for requesting pose of ee and joint values
        self.req_pose_joint = rospy.Publisher("/joint_ee_pose_req", 
                                        Empty, queue_size=1)
        rospy.sleep(1)

    def ack_clbck(self, msg):
        '''
        callback function for flag plan retrun value
        '''
        self.flag_plan = msg.data # storing the data part (expected: Bool)

    def pose_joint_clbck(self, msg):
        '''
        callback function for pose ee and joint angles
        '''
        self.pose_and_joint_angles = eval(msg.data) # storing the data part (expected: String) 
                                                    # and evaling for python data types
        self.sample_pose = Pose() # creating Pose instance for converting manually the data

        # parsing position 
        self.sample_pose.position.x        = self.pose_and_joint_angles[0][0]
        self.sample_pose.position.y        = self.pose_and_joint_angles[0][1]
        self.sample_pose.position.z        = self.pose_and_joint_angles[0][2] 

        # parsing orientation
        self.sample_pose.orientation.x     = self.pose_and_joint_angles[0][3]
        self.sample_pose.orientation.y     = self.pose_and_joint_angles[0][4]
        self.sample_pose.orientation.z     = self.pose_and_joint_angles[0][5]
        self.sample_pose.orientation.w     = self.pose_and_joint_angles[0][6]

        self.pose_and_joint_angles[0] = self.sample_pose  # Replacing the first element with actual pose format


    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Setting joint angles
        ---
        Input : Joint angles in Float32MultiArray format
        Output: Flag of execution (0 -> failed, 1 -> success)
        '''
        data_for_float32 = Float32MultiArray()        # Float32Array data type has two sub-data types
        data_for_float32.data = arg_list_joint_angles # storing in data part

        self.pub_joint_angles.publish(data_for_float32) # publishes joint angles on /set_joint_value_target_wait_topic
        rospy.wait_for_message("/joint_pose_ack", Bool, timeout=None) # wait till acknowledgment is recieved

        if (self.flag_plan == True): #Logging flag
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return self.flag_plan # returning the flag of plan on std true-false logic

    def set_pose(self, arg_pose):
        '''
        Setting pose values
        ---
        Input : pose values in Pose format
        Output: Flag of execution (0 -> failed, 1 -> success)
        '''
        self.pub_pose_values.publish(arg_pose) # publishes joint angles on /set_joint_value_target_wait_topic
        rospy.wait_for_message("/joint_pose_ack", Bool, timeout=None) # wait till acknowledgment is recieved

        if (self.flag_plan == True): #Logging flag
            rospy.loginfo(
                '\033[94m' + ">>> set_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_pose() Failed." + '\033[0m')

        return self.flag_plan # returning the flag of plan on std true-false logic


    def print_pose_ee_joint(self): # but outputs both pose and joint angles
                             
        '''
        For accessing pose and joint angles of bot
        ---
        Output: List in format of:
                - index 1: pose list of end-effector   
                - index 2: joint angles list in the angle order 
        '''
        
        self.req_pose_joint.publish() # Publishing as signal to request print the pose and joint angles
        rospy.wait_for_message("/joint_ee_pose", String, timeout=None) # wait till acknowledgment is recieved
        print("/joint_ee_pose topic is now available!!!!")

        try:
            print("End-effector pose: \n", self.pose_and_joint_angles[0])
            print("Joint angle values: \n", self.pose_and_joint_angles[1])

            pose_values = self.pose_and_joint_angles[0] # Storing the first part of the list
            q_x = pose_values.orientation.x             # Parsing the orientation values
            q_y = pose_values.orientation.y
            q_z = pose_values.orientation.z
            q_w = pose_values.orientation.w
            quaternion_list = [q_x, q_y, q_z, q_w]

            (roll, pitch, yaw) = euler_from_quaternion(quaternion_list) # converting the quaternion to euler

            # Logging the end-effector pose and RPY
            rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                        "x: {}\n".format(pose_values.position.x) +
                        "y: {}\n".format(pose_values.position.y) +
                        "z: {}\n\n".format(pose_values.position.z) +
                        "roll: {}\n".format(roll) +
                        "pitch: {}\n".format(pitch) +
                        "yaw: {}\n".format(yaw) +
                        '\033[0m')


        except Exception as e:
            rospy.logerr("Exception occurred while executing print function: ", str(e))
            self.pose_and_joint_angles = [self.sample_pose, [0,0,0,0,0,0]]  # returning empty values

        return self.pose_and_joint_angles

    def gripper_control(self, logic_level):
        '''
        Controlling gripper to open and close where:
        0 -> Open
        1 -> close
        '''
        rospy.wait_for_service('/ur_hardware_interface/set_io')                # wait for service availabilty
        spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO) # proceed to defining proxy service
        spawn_srv(fun=1, pin=16, state=logic_level) # passing the logic to pin 16 (gripper connection)
        rospy.sleep(4) # wait as it non-blocking call
        rospy.logdebug("Gripper control executed successfully: " + str(logic_level))


    # Destructor
    def __del__(self):
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def transform_pose(self, src):
        try:
            # self.listener.waitForTransform("ebot_base" , "pepper" , rospy.Time() , rospy.Duration(4.0))
            rospy.loginfo("in the transform function")
            transform = []
            #trans = self.tf_buffer.lookup_transform('ebot_base' , 'fruit_red' , rospy.Time())
            listener = tf.TransformListener()
            listener.waitForTransform("ebot_base", src, rospy.Time(), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform('ebot_base', src,rospy.Time())


            #print("TRANSFORM :" , trans, rot)
            rospy.loginfo(trans)
            return trans, rot

        except:

            return [],[]

    # def handle_turtle_pose(self, x , y, z ) :
    #     br = tf.TransformBroadcaster()
    #     br.sendTransform((x , y , z),
    #                     (0,0,0,1),
    #                     rospy.Time.now(),
    #                     "lmaooooo",
    #                     "ebot_base")
    #     print(x ,y ,z )                
    #     print("Transform broadcast")

def main():


    rospy.init_node("peppercatcher", anonymous=True)
    try:
        ms = Ur5Moveit()
        flag1 = False 
        attempt = 0 
        attempt2 = 0
        arm_rotation = 0
        flag2 = False
        flag = False

        # ms.pluck_pub.publish(String("False"))

        red_drop_pose =  [-0.5719807786855355, -1.9448688909752914, 1.2759203567840753, 0.7128009359972172, 1.5576425501842497, -1.4519208990383037]
        yellow_drop_pose = [0.07924969282430361, -1.9447363113586276, 1.2759449175686086, 0.712764349676509, 1.5574983074550204, -1.452046107841527]


        # right_inter_pose = [-1.5752570936587817, -2.0767219707225406, 1.3640831080452562, 0.7127025913258764, 1.5575267447673973, -1.451998526717846]
        # left_inter_pose = [1.5576133728027344, -2.8006861845599573, 1.6631155014038086, 1.2057710886001587, 1.5707075595855713, -1.5712140242206019]

        left_inter_pose = [math.radians(90),math.radians(-153),math.radians(90),math.radians(63),math.radians(80),math.radians(-90)]
        right_inter_pose = [math.radians(-90),math.radians(-153),math.radians(90),math.radians(63),math.radians(80),math.radians(-90)]

        red_drop_1 = [math.radians(-30),math.radians(-124),math.radians(131),math.radians(0),math.radians(84),math.radians(-91)]
        yellow_drop_1 = [math.radians(18),math.radians(-124),math.radians(131),math.radians(0),math.radians(84),math.radians(-91)]


        if arm_rotation == 0:

            pose = left_inter_pose
            offset_interpose = 0.33
            offset_pose = 0.275
            orientation_w = 0.5
            orientation_z = 0.5
            # pose_x = 0.0004508026344099538
            # pose_y = -0.7052359925739942
            # pose_z = 0
            # pose_w = 0.008059155505845033
        else :

            pose = right_inter_pose
            offset_interpose = -0.33
            offset_pose = - 0.275
            orientation_w = -0.5
            orientation_z = -0.5
            # pose_x = -0.006533642089305424
            # pose_y = 0.7477652292772122
            # pose_z = -1
            # pose_w = 0.6639039569546898

        ms.print_pose_ee_joint()
        ms.set_joint_angles(pose)
        # ms.gripper_control(0)
        # rospy.sleep(2)

        while True:

            transform_yellow, rot_yellow=ms.transform_pose("Yellow_pepper")
            transform_red, rot_red=ms.transform_pose("Red_pepper")
            ms.print_pose_ee_joint()

            # red_pose_interpose = geometry_msgs.msg.Pose()
            # # red_pose_interpose.orientation.x =  0
            # red_pose_interpose.orientation.y = 1
            # # red_pose_interpose.orientation.z = 1.1528102031688827

            # while not flag :
            #     rospy.loginfo("into the red function")

            #     flag = ms.set_pose(red_pose_interpose)

            #     rospy.loginfo("outside of the red pose")

            if len(transform_red)!=0:

                rospy.loginfo("Into the red pepper")
                # ms.handle_turtle_pose(transform_red[0] , transform_red[1] , transform_red[2])
                
                attempt2 = 0

                red_pose_interpose = geometry_msgs.msg.Pose()
                red_pose_interpose.position.x = round(transform_red[0] ,2 ) 
                red_pose_interpose.position.y = round(transform_red[1] ,2 ) - offset_interpose
                red_pose_interpose.position.z = round(transform_red[2] ,2 ) 
                red_pose_interpose.orientation.x = -0.5
                red_pose_interpose.orientation.y = 0.5
                red_pose_interpose.orientation.z = orientation_z
                red_pose_interpose.orientation.w = orientation_w

                red_pose = geometry_msgs.msg.Pose()
                red_pose.position.x = round(transform_red[0] ,2 ) 
                red_pose.position.y = round(transform_red[1] ,2 ) - offset_pose
                red_pose.position.z = round(transform_red[2] ,2 ) 

                red_pose.orientation.x = -0.5
                red_pose.orientation.y = 0.5
                red_pose.orientation.z = orientation_z
                red_pose.orientation.w = orientation_w


                while not flag2 and attempt2 < 11 :

                    rospy.loginfo("going to the red pose ")
                    if attempt2 < 6 or not second_pose :
                        second_pose = ms.set_pose(red_pose_interpose)
                        attempt2 += 1

                    if attempt2 >= 6 and attempt2 < 11 :
                        rospy.loginfo("going close to the pose")
                        flag2 = ms.set_pose(red_pose)
                        attempt2 += 1 

                if flag2 :
                    print("fruit_red_Plucked")

                ms.gripper_control(1)
                ms.set_joint_angles(red_drop_1) 
                ms.gripper_control(0)
                # ms.set_joint_angles(pose)
                ms.pluck_pub.publish("Move")

                if flag2 :
                    print("fruit_red Dropped in red_box")

            
            if len(transform_yellow)!=0:

                rospy.loginfo("Into yellow pepper")

                yellow_pose = geometry_msgs.msg.Pose()
                yellow_pose.position.x = round(transform_yellow[0] ,2 ) 
                yellow_pose.position.y = round(transform_yellow[1] ,2 ) - offset_interpose
                yellow_pose.position.z = round(transform_yellow[2] ,2 ) 
                # yellow_pose.orientation.z = pose_z


                yellow_pose.orientation.x = -0.5
                yellow_pose.orientation.y= 0.5
                yellow_pose.orientation.z= orientation_z
                yellow_pose.orientation.w = orientation_w

                yellow_inter_pose = geometry_msgs.msg.Pose()
                yellow_inter_pose.position.x = round(transform_yellow[0] ,2 ) 
                yellow_inter_pose.position.y = round(transform_yellow[1] ,2 ) - offset_pose
                yellow_inter_pose.position.z = round(transform_yellow[2] ,2 ) 
                # yellow_inter_pose.orientation.z = pose_z

                yellow_inter_pose.orientation.x = -0.5
                yellow_inter_pose.orientation.y = 0.5
                yellow_inter_pose.orientation.z = orientation_z                
                yellow_inter_pose.orientation.w = orientation_w

                #print(detect_pose)    
                # rospy.loginfo("Trying to go to the pose")

                while not flag1 and attempt < 11 :

                    if attempt < 6:
                        first_pose = ms.set_pose(yellow_inter_pose)
                        attempt += 1
                        rospy.loginfo("Unable to reach")
                    if attempt >= 6 and  attempt < 11:
                        flag1 = ms.set_pose(yellow_pose)
                        attempt += 1
                        rospy.loginfo("Reached the Pose")
                        #rospy.loginfo(attempt)

                if flag1 :
                    print("fruit_yellow_Plucked")
                
                ms.gripper_control(1)
                ms.set_joint_angles(yellow_drop_1) 
                ms.gripper_control(0)
                # ms.set_joint_angles(pose)
                # ms.pluck_pub.publish("True")
                ms.pluck_pub.publish("Move")

                if flag1 :
                    print("fruit_yellow Dropped in yellow_box")

            

            
    except Exception as e:
        print("Error:", str(e))    


if __name__=="__main__" :
    main()
    rospy.spin()




