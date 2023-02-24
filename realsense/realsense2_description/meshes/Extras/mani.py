#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 
import tf
import tf2_msgs.msg

from Pepper_finder import PerceptionStack
import tf2_ros
#from transform_frames import TransformFrames
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from std_msgs.msg import Header

from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs

class FixedTFbroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
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


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


    def transform_pose(self):
        # self.listener.waitForTransform("ebot_base" , "pepper" , rospy.Time() , rospy.Duration(4.0))
        try:
            trans = self.tf_buffer.lookup_transform('ebot_base' , 'pepper' , rospy.Time(0))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            x1 = trans.transform.rotation.x
            y1 = trans.transform.rotation.y
            z1 = trans.transform.rotation.z
            w = trans.transform.rotation.w


            return x , y , z , x1 , y1 , z1 , w

        except:
            pass


    def main(self):

        rospy.loginfo("started")
       
        detect_pose = [math.radians(100),math.radians(-25),math.radians(-54),math.radians(82),math.radians(-7),math.radians(0)]
        inter_pose = [math.radians(-80),math.radians(0),math.radians(0),math.radians(0),math.radians(0),math.radians(0)]
        inter_pose2 =  [math.radians(-257),math.radians(-23),math.radians(-60),math.radians(86),math.radians(0),math.radians(-1)]
        yellow_basket =  [math.radians(11),math.radians(3),math.radians(9),math.radians(0),math.radians(5),math.radians(0)]
        close=[math.radians(23)]
        result = self.transform_pose()
        
        if result != None :
            detect_pose = geometry_msgs.msg.Pose()
            detect_pose.position.x = result[0]
            detect_pose.position.y = result[1]
            detect_pose.position.z = result[2]

            # detect_pose.orientation.x = result[3]
            # detect_pose.orientation.y = result[4]
            # detect_pose.orientation.z = result[5]
            # detect_pose.orientation.w = result[6]

        detect_pose1 = geometry_msgs.msg.Pose()
        detect_pose1.position.x = 0.10969103247684046
        detect_pose1.position.y = -0.343399873902921
        detect_pose1.position.z =  1.0550548568280893

        detect_pose1.orientation.x = -0.14770761462167648
        detect_pose1.orientation.y = 0.9886845985686313
        detect_pose1.orientation.z = -0.026166747456944396
        detect_pose1.orientation.w = 0.000725578033768894 

        while not rospy.is_shutdown():
            
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "camera_depth_frame2"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "pepper"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = self.z
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1            
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

            flag = False
            flag1 = False

            self.set_joint_angles(inter_pose)
            self.set_joint_angles(inter_pose2)

            # while not flag1:
            #     flag1 = self.go_to_pose(detect_pose1)
            #     print("trying to get the pose")

            rospy.loginfo("perception value")
            ps = PerceptionStack()
            # pose1 = ps.rgb_image_processing()
            # depth1 = ps.depth_image_processing(pose1)
            # rospy.loginfo("adding the transform")
            position1 = ps.output()

            rospy.loginfo("added the transform")

            self.x = position1[0][0]
            self.y = position1[0][1]
            self.z = position1[0][2]
            
            print(self.x ,self.y , self.z)
        
 
            # self.set_joint_angle_1(close)

            # self.set_joint_angles(yellow_basket)

            # self.go_to_predefined_pose("open", 2)




if __name__ == '__main__':
    rospy.init_node('manistack')
    tfb = FixedTFbroadcaster()
    rospy.loginfo("started the main")
    tfb.main()




