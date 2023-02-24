#! /usr/bin/env python3

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class FixedTFbroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.tf_buffer = tf2_ros.buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.x = 0
        self.y = 0        
        self.z = 0
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

    def new_frame( x , y , z , self):


        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "camera_depth_frame2"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "pepper"
        t.transform.translation.x = x 
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1            
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def publishing_the_trans():
        pass
        

    def main(self):

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



if __name__ == '__main__':
    rospy.init_node('fixed_inint_node')



    rospy.spin()
