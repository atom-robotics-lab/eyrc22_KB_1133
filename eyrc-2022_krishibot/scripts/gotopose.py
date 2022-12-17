#! /usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math 
import tf

class Ur5Moveit:
    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)
        self.listener = tf.TransformListener()
        
        self._planning_group1 = "arm"
        self._planning_group2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group1 = moveit_commander.MoveGroupCommander(self._planning_group1)
        self._group2 = moveit_commander.MoveGroupCommander(self._planning_group2)

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
        self._group1.set_pose_reference_frame("odom")
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names1) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
    

    def go_to_pose(self, arg_pose):
        
        pose_values = self._group1.get_current_pose()
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        # self._group.setPlanningTime(10)
        self._group1.set_pose_target(arg_pose)
        # self._group1.setPlanningTime(10)
        flag_plan = self._group1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        plan = self._group1.plan()

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group2.set_joint_value_target(arg_list_joint_angles)
        self._group2.plan()
        flag_plan = self._group2.go(wait=True)

        list_joint_values = self._group2.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group2.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)   


    # def go_to_predefined_pose(self, arg_pose_name):
    #     rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
    #     self._group2.set_named_target(arg_pose_name)
    #     plan = self._group2.plan()
    #     goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    #     goal.trajectory = plan
    #     self._exectute_trajectory_client.send_goal(goal)
    #     self._exectute_trajectory_client.wait_for_result()
    #     rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


    def main(self):

        ur5 = Ur5Moveit()

        lst_joint_angles_1 = [math.radians(46)]
        lst_joint_angles_2 = [math.radians(0)]
        ur5_pose_1 = geometry_msgs.msg.Pose()

        # ur5_pose_1.position.x = -0.12218416594171036
        # ur5_pose_1.position.y = 0.6816194710660304
        # ur5_pose_1.position.z = 0.852740509065047
        # ur5_pose_1.position.x = -0.20959908029056795
        # ur5_pose_1.position.y = 0.8827731484016123
        # ur5_pose_1.position.z = -0.6189922662354054
        ur5_pose_1.position.x = 0.11
        ur5_pose_1.position.y = 0.68
        ur5_pose_1.position.z = 1.290

        ur5_pose_1.orientation.x = -0.1134244319123348
        ur5_pose_1.orientation.y = 0.9931656428888161
        ur5_pose_1.orientation.z = -0.02495704095855228
        ur5_pose_1.orientation.w = 0.011578002297534336

# - Translation: [0.026, 0.704, 1.055]
# - Translation: [-0.375, 0.518, 0.793]

        # ur5_pose_2 = geometry_msgs.msg.Pose()
        # ur5_pose_2.position.x = 0.30881936338021004
        # ur5_pose_2.position.y = 0.34894018085231596
        # ur5_pose_2.position.z = 1.302535860885369
        # ur5_pose_2.orientation.x = 0.06784305093041833
        # ur5_pose_2.orientation.y = -0.7038443832629431
        # ur5_pose_2.orientation.z = 0.06780309201308238
        # ur5_pose_2.orientation.w = 0.7038488085541703

        # ur5_pose_4 = geometry_msgs.msg.Pose()
        # ur5_pose_4.position.x = 0.30881936338021004
        # ur5_pose_4.position.y = 0.44894018085231596
        # ur5_pose_4.position.z = 1.302535860885369
        # ur5_pose_4.orientation.x = 0.06784305093041833
        # ur5_pose_4.orientation.y = -0.7038443832629431
        # ur5_pose_4.orientation.z = 0.06780309201308238
        # ur5_pose_4.orientation.w = 0.7038488085541703
        # ur5_pose_3 = geometry_msgs.msg.Pose()
        # ur5_pose_3.position.x = -0.115508484882963427
        # ur5_pose_3.position.y = 0.4569984058257204                   
        # ur5_pose_3.position.z = 0.8456052990857847
        # ur5_pose_3.orientation.x = -0.02580152243737326
        # ur5_pose_3.orientation.y = 0.6818494240900229
        # ur5_pose_3.orientation.z = 0.032966579544718426
        # ur5_pose_3.orientation.w = 0.7302936730803161
        # ur5_pose_6 = geometry_msgs.msg.PoseStamped()
        
        # ur5_pose_6.position.x = 0.04 + 0.183
        # ur5_pose_6.position.y = -0.117 + 0.8 - 0         
        # ur5_pose_6.position.z = 1.02 + 0.49
        # arg_pose_world = self.listener.transformPose("world" , )

        # ur5_pose_6.orientation.x = -0.02580152243737326
        # ur5_pose_6.orientation.y = 0.6818494240900229
        # ur5_pose_6.orientation.z = 0.032966579544718426
        # ur5_pose_6.orientation.w = 0.7302936730803161
        # ur5_pose_3.position.x = -0.27351528427513743
        # ur5_pose_3.position.y = 0.5038260116717826                   
        # ur5_pose_3.position.z = 0.8443492216291616
        # ur5_pose_3.orientation.x = 0.19401890418971424
        # ur5_pose_3.orientation.y = 0.6737909950620148
        # ur5_pose_3.orientation.z = -0.2228272765232043
        # ur5_pose_3.orientation.w = 0.677281599209379
        # ur5_pose_3.position.x = -0.0018198502094235306
        # ur5_pose_3.position.y = 0.47492744186314645                   
        # ur5_pose_3.position.z = 0.8204475601938356
        # ur5_pose_3.orientation.x = 0.005415648508630287
        # ur5_pose_3.orientation.y = 0.7165730652636353
        # ur5_pose_3.orientation.z = 0.036201815880803914
        # ur5_pose_3.orientation.w = 0.6965508893231286
        # ur5_pose_5 = geometry_msgs.msg.Pose()
        # ur5_pose_5.position.x = 0.005508484882963427
        # ur5_pose_5.position.y = 0.4569984058257204                   
        # ur5_pose_5.position.z = 0.8456052990857847
        # ur5_pose_5.orientation.x = -0.02580152243737326
        # ur5_pose_5.orientation.y = 0.6818494240900229
        # ur5_pose_5.orientation.z = 0.032966579544718426
        # ur5_pose_5.orientation.w = 0.7302936730803161
        # ur5_pose_3.orientation.x = 0.04997915342387482
        # ur5_pose_3.orientation.y = 3.9799747470500266e-05
        # ur5_pose_3.orientation.z = -0.9987499437230536
        # ur5_pose_3.orientation.w = 0.0007953315097043137

        # ur5_pose_3.orientation.x = 0.0461656649113825
        # ur5_pose_3.orientation.y = 0.018449194957275778
        # ur5_pose_3.orientation.z = -0.9253275620780133
        # ur5_pose_3.orientation.w = 0.3758952798951847

    #   x: 0.30881936338021004
    #   y: 0.44894018085231596
    #   z: 1.312535860885369
    # orientation: 
    #   x: 0.06784305093041833
    #   y: -0.7038443832629431
    #   z: 0.06780309201308238
    #   w: 0.7038488085541703

    #  x: 0.23686436285183668
    #   y: 0.46979603016598936
    #   z: 1.2335819417323315
    # orientation: 
    #   x: 0.02165337217485133
    #   y: -0.7831686763058593
    #   z: -0.1281843538121597
    #   w: 0.6080680285663312


        while not rospy.is_shutdown():
            print("first pose")
            # ur5.go_to_joint()
            # ur5.go_to_pose(ur5_pose_1)
            # rospy.sleep(9)
            print("ended first pose")

            print("second pose")
            flag3 = False
            flag4 = False
            flag1 = False
            flag2 = False
            flag5 = False
            flag6 = False
            
            while not flag3:
                
                flag3 = ur5.go_to_pose(ur5_pose_1)


            # while not flag4:
            #     flag4 = ur5.go_to_pose(ur5_pose_2)
            #     ur5.set_joint_angles(lst_joint_angles_1)
            #     rospy.sleep(1) 

            # while not flag5:
            #     flag5 = ur5.go_to_pose(ur5_pose_1)
            #     rospy.sleep(1)
            #     ur5.set_joint_angles(lst_joint_angles_2)


            # while not flag1:
            #     flag1 = ur5.go_to_pose(ur5_pose_3)
            #     rospy.sleep(1)

            # print("ended 2nd pose")

            # while not flag2:
            #     flag2 = ur5.go_to_pose(ur5_pose_5)
            #     rospy.sleep(2)
            #     ur5.set_joint_angles(lst_joint_angles_1)
            #     rospy.sleep(2)

            # while not flag6:

            #     flag6 = ur5.go_to_pose(ur5_pose_1)
            #     rospy.sleep(1)
            #     ur5.set_joint_angles(lst_joint_angles_2)

            
            # ur5.go_to_predefined_pose("close")





if __name__ == '__main__':
    urf = Ur5Moveit()
    urf.main()
