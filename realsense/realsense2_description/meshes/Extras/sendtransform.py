#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

def handle_transform(input_pose , from_frame , to_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pose_stamped = tf2_geometry_msgs.PoseStamped()

    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()
    try:
        output_pos = tf_buffer.transform(pose_stamped , to_frame , rospy.Duration(1))
        return output_pos.pose
    except:
        print("lol")
    
    rospy.init_node("transform_test")
    my_pose = Pose()
    my_pose.position.x = 0
    my_pose.position.y = 0
    my_pose.position.z = 0
    my_pose.orientation.x = 0
    my_pose.orientation.y = 0
    my_pose.orientation.z = 0
    my_pose.orientation.w = 0

    tarnsformed_pose = handle_transform(my_pose , "fixtures" , "ebot_base")
    print(tarnsformed_pose)
    
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "ebot_base"
    # t.child_frame_id = turtlename
    # t.transform.translation.x = 0
    # t.transform.translation.y = 0
    # t.transform.translation.z = 0.0
    # q = tf_conversions.transformations.quaternion_from_euler(0 , 0 , 0)
    # t.transform.rotation.x = q[0]
    # t.transform.rotation.y = q[1]
    # t.transform.rotation.z = q[2]
    # t.transform.rotation.w = q[3]

#     br.sendTransform(t)

# if __name__== '__main__':

#     rospy.init_node('tf2_turtle_broadcaster')
#     rospy.Subscriber('/%s/pose1', handle_transform("odom"))
#     rospy.spin()
    