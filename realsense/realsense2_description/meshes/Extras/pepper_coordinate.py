#! /usr/bin/env python3

from Pepper_finder import PerceptionStack

import rospy
class pepper_coordinate:
    
    def __init__(self):
        self.fx = 554.3827128226441
        self.fy = 554.3827128226441
        self.cx = 320.5
        self.cy = 240.5

    def transform(pose2 , depth_value , self):
        rospy.loginfo("starting the transform")
        transform = []

        for i in range (len(pose2)):
            rospy.loginfo("accessing the depth value")
            current_pose , current_depth = pose2[i] , depth_value[i]

            X = current_depth * ((current_pose[0]-self.cx)/self.fx)
            Y = current_depth * ((current_pose[1]-self.cy)/self.fy)
            Z = current_depth

            result = (X , Y , Z)
            transform.append(result)
        
        return transform 
    def main(self):
        rospy.loginfo("calling the main")
        ps = PerceptionStack()
        pose1 = ps.rgb_image_processing()
        depth_value = ps.depth_image_processing(pose1)
        
        print(self.transform(pose1 , depth_value ))


if __name__ == '__main__':
    rospy.init_node('manistack')
    rospy.loginfo("calling the class")
    tfb = pepper_coordinate()

    rospy.loginfo("started the main")
    
    tfb.main()
