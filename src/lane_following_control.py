#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Int32MultiArray, Float32

# TODO: import ROS msg types and libraries
CENTROID_TOPIC_NAME = '/centroid'


class LineFollow(object):
    def __init__(self):
        self.centroid_subscriber = rospy.Subscriber(CENTROID_TOPIC_NAME, Float32, self.controller)
        self.linefollow_publisher = rospy.Publisher('/vesc')

        self.speed = 0.5
        self.angle = 0
        # TODO: create ROS subscribers and publishers.


    def centroid_callback(self, msg):
        # TODO: update current angle
        



def main():
    rospy.init_node('line_following_control_node')
    sn = LineFollow()
    rospy.spin()
if __name__ == '__main__':
    main()