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
    gp = GazeboLinkPose('ebot::wrist_3_link')
    publish_rate = 10
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
# if __name__ == '__main__':
#   q = quaternion_from_euler(0 , 0.1 , -3.14)
#   print ("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))
  # x: 0.1829898898146679
  # y: 0.8023376071527116
  # z: 0.4900050795714763
  #   x: 0.29316420837193224
  # y: 0.7451849227233505
  # z: 1.5100408694070029

#   x: 0.1080323273061396     −0.08572708
#   y: 0.6728762748777448     0.038091966
#   z: 1.5100490227554684     −0.458772815
#   x: -0.16499046731230224      −0.208819898 
#   y: 0.6918976388807493        -0.178
#   z: 1.0300351941596693        −0.23813027
# [-0.382876169998026, 0.529100098667977, 0.839030203193756
# [0.022305246893086356, 0.7109682406028226, 1.0512762074530082]
# [-0.37381036552333524, 0.5138968160213401, 0.7919049246412744]

  x: 0.5168820852871454   
  y: 0.07053851457126975
  z: 1.045907913154687


  x: 0.2507128890970452   
  y: 0.10902520625655877
  z: 0.8251134529581701


  x: -0.19992935324638775
  y: 0.6699861816419682
  z: 0.8300263533896246
orientation: 
  x: 0.00026068359927457414
  y: 0.00037084824013296116
  z: 0.0001444496332126817
  w: 0.9999998868249672
position: 
  x: -0.0698393462065209
  y: 0.6761307079837067
  z: 1.0500157560917838
orientation: 
  x: -0.0005187140601250822
  y: 9.524317652252409e-05
  z: 0.011372411882715367
  w: -0.9999351929562396