#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 
import tf
from Pepper_finder import PerceptionStack
#import tf2_ros
#from transform_frames import TransformFrames
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from std_msgs.msg import Header

from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped
from std_msgs.msg import Header
import tf2_ros, tf2_geometry_msgs



class TransformFrames():
    def __init__(self):
        ''' Create a buffer of transforms and update it with TransformListener '''
        self.tfBuffer = tf2_ros.Buffer()           # Creates a frame buffer
        tf2_ros.TransformListener(self.tfBuffer)   # TransformListener fills the buffer as background task
    
    def get_transform(self, source_frame, target_frame):
        ''' Lookup latest transform between source_frame and target_frame from the buffer '''
        try:
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2) )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f'Cannot find transformation from {source_frame} to {target_frame}')
            raise Exception(f'Cannot find transformation from {source_frame} to {target_frame}') from e
        return trans     # Type: TransformStamped

    def pose_transform(self, pose_array, target_frame='odom'):
        ''' pose_array: will be transformed to target_frame '''
        trans = self.get_transform( pose_array.header.frame_id, target_frame )
        new_header = Header(frame_id=target_frame, stamp=pose_array.header.stamp) 
        pose_array_transformed = PoseArray(header=new_header)
        for pose in pose_array.poses:
            pose_s = PoseStamped(pose=pose, header=pose_array.header)
            pose_t = tf2_geometry_msgs.do_transform_pose(pose_s, trans)
            pose_array_transformed.poses.append( pose_t.pose )
        return pose_array_transformed

    def get_frame_A_origin_frame_B(self, frame_A, frame_B ):
        ''' Returns the pose of the origin of frame_A in frame_B as a PoseStamped '''
        header = Header(frame_id=frame_A, stamp=rospy.Time(0))        
        origin_A = Pose(position=Point(0.,0.,0.), orientation=Quaternion(0.,0.,0.,1.))
        origin_A_stamped = PoseStamped( pose=origin_A, header=header )
        pose_frame_B = tf2_geometry_msgs.do_transform_pose(origin_A_stamped, self.get_transform(frame_A, frame_B))
        return pose_frame_B



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
        rospy.loginfo(pose_values)
        # self._group.setPlanningTime(10)
        self._group1.set_pose_target(arg_pose)
        #self._group1.setPlanningTime(10)
        flag_plan = self._group1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
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

def callback():
    tf2Broadcast = tf2_ros.TransformBroadcaster()
    tf2Stamp = geometry_msgs.msg.TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = 'camera_depth_frame2'
    tf2Stamp.child_frame_id = 'ebot_base'
    tf2Stamp.transform.translation = (0.04, -0.117, 1.02)
    tf2Stamp.transform.rotation = (0.0, 0.0, -3.14)
    tf2Broadcast.sendTransform(tf2Stamp)


def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Time(0))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def find_transforms(pose, depth_val) :
    transforms = []
    fx, fy = [554.3827128226441, 554.3827128226441]
    cx, cy = [320.5, 240.5]

    #tf = TransformFrames()
    tf_buffer = tf2_ros.Buffer()
    tf_listener=tf2_ros.TransformListener(tf_buffer)

    #pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))


    for i in range(len(pose)) :
        current_pose, current_depth = pose[i], depth_val[i]

        X = current_depth * ((current_pose[0]-cx)/fx)
        Y = current_depth * ((current_pose[1]-cy)/fy)
        Z = current_depth

        #pose_array.poses.append(Pose(position = Point(X, Y, Z), orientation =  Quaternion(0,0,0,0)))

        #transforms.append([X,Y,Z])

        p=PoseStamped()
        p.header.stamp=rospy.Time()
        p.header.frame_id='camera_depth_frame2'
        p.pose.position.x= X - 0.06
        p.pose.position.y= Y + 0.25
        p.pose.position.z= Z + 0.545
        p.pose.orientation.x=0
        p.pose.orientation.y=0
        p.pose.orientation.z=0
        p.pose.orientation.w=0

        res=tf_buffer.transform(p,'ebot_base',timeout=rospy.Duration(5))
        transforms.append(res)



    #res=tf_buffer.transform(pose_array,'ebot',timeout=rospy.Duration(5))

    #transforms = tf.pose_transform(pose_array, "ebot_base")

    return transforms
    #return pose_array


def main():
    
    ur5 = Ur5Moveit()
    ps = PerceptionStack() 

    detect_pose = [math.radians(100),math.radians(-25),math.radians(-54),math.radians(82),math.radians(-7),math.radians(0)]
    inter_pose = [math.radians(-80),math.radians(0),math.radians(0),math.radians(0),math.radians(0),math.radians(0)]
    inter_pose2 =  [math.radians(-244),math.radians(17),math.radians(1),math.radians(-25),math.radians(0),math.radians(-180)]
    yellow_basket =  [math.radians(11),math.radians(3),math.radians(9),math.radians(0),math.radians(5),math.radians(0)]
    close=[math.radians(23)]
    detect_pose = geometry_msgs.msg.Pose()
    detect_pose.position.x = 0.10969103247684046
    detect_pose.position.y = -0.343399873902921
    detect_pose.position.z =  1.0550548568280893

    detect_pose.orientation.x = -0.14770761462167648
    detect_pose.orientation.y = 0.9886845985686313
    detect_pose.orientation.z = -0.026166747456944396
    detect_pose.orientation.w = 0.000725578033768894


    
   
    while not rospy.is_shutdown():        
        flag = False
        ur5.set_joint_angles(inter_pose)
        ur5.go_to_pose(detect_pose)
        rospy.sleep(3)
        pose = ps.rgb_image_processing()
        depth_val = ps.depth_image_processing(pose)

        transforms = find_transforms(pose, depth_val)
        #transforms = transform_pose(transforms, "camera_depth_frame2", "odom")
        #print("TRANSFORMS  :", transforms)

        for i in range(len(transforms)) :
            ur5.set_joint_angles(inter_pose2)

            ur5.go_to_predefined_pose("open", 2)
            #while not flag: 
                #transforms[i].pose.position.z -= 0.1
                #transforms[i].pose.position.y -= 0.1
            ur5.go_to_pose(transforms[i])
            '''flag = False
            while not flag: 
                transforms[i].pose.position.z += 0.1
                #transforms[i].pose.position.y += 0.1
                flag = ur5.go_to_pose(transforms[i])'''
            #print(transforms[i])
            #transforms[i].pose.position.y -= 0.15
            #ur5.go_to_pose(transforms[i])
            #flag = False

            #while not flag : 
            ur5.set_joint_angle_1(close)

            ur5.set_joint_angles(yellow_basket)

            ur5.go_to_predefined_pose("open", 2)


            
            #ur5.go_to_pose([])

        
        
    del ur5


if __name__ == '__main__':

    #rospy.init_node('manistack')
    
    main()

