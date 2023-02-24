#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
class GazeboLinkPose:
  link_name = ''
  link_pose = Pose()
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]
    except ValueError:
      pass

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose('ebot::ebot_base')
    publish_rate = 10
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
#   x: 5.347828869426875
#   y: -0.04836346242463708
#   z: 0.15363389080593318
# position: 
#   x: 5.423852263866617
#   y: -0.7015709922162884
#   z: 1.4099276235831348
