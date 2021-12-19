#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point32


RACECAR_DETECTION_NODE_NAME = 'racecar_pose_node'
CAMERA_TOPIC_NAME = '/zed2/zed_node/rgb/image_rect_color'
CAMERA_INFO_TOPIC_NAME = '/zed2/zed_node/rgb/camera_info'
BOX_CENTER_TOPIC_NAME = 'box_center'
RACECAR_POSITION_TOPIC_NAME = 'racecar_position'


def decodeImage(data, height, width):
    decoded = np.fromstring(data, dtype=np.uint8)
    decoded = decoded.reshape((height, width, 4))
    return decoded[:, :, :3]


class RacecarPoseEstimate:
    def __init__(self):
        # Initialize node and create publishers/subscribers
        self.init_node = rospy.init_node(RACECAR_DETECTION_NODE_NAME, anonymous=False)
        self.camera_subscriber = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, self.image_callback)
        self.camera_info_subscriber = rospy.Subscriber(CAMERA_INFO_TOPIC_NAME, CameraInfo, self.camera_info_callback)
        self.box_center_subscriber = rospy.Subscriber(BOX_CENTER_TOPIC_NAME, Point32, self.box_center_callback)
        self.racecar_position_publisher = rospy.Publisher(RACECAR_POSITION_TOPIC_NAME, Point32, queue_size=1)

        # variable for storing data from messages
        self.intrinsic_matrix = None
        self.frame = None
        self.box_center_x = None
        self.box_center_y = None

        # manually defined transformation from camera frame to world frame (ego racecar frame)
        self.T_cam_2_world = np.array([[0.085], [0.3], [0.14]])
        rotation_radian = np.radians(-112)
        self.R_cam_2_world = np.array([[1, 0, 0],
            [0, np.cos(rotation_radian), -np.sin(rotation_radian)], 
            [0, np.sin(rotation_radian), np.cos(rotation_radian)]])

    def image_callback(self, data):
        # Image processing from rosparams
        self.frame = decodeImage(data.data, 376, 672)
        # cv2.imshow('frame', self.frame)
        # # plotting results
        # cv2.waitKey(1)

    def box_center_callback(self, data):
        self.box_center_x = int(data.x * 672)
        self.box_center_y = int(data.y * 376)
        print('Center: {}, {}'.format(self.box_center_x, self.box_center_y))
        if self.frame is not None:
            self.frame = cv2.circle(self.frame, (self.box_center_x, self.box_center_y), 7, (255, 0, 0), -1)
            cv2.imshow('frame', self.frame)
            cv2.waitKey(1)

        pixel_coord = np.array([[self.box_center_x], [self.box_center_y], [1]])

        if self.intrinsic_matrix is not None:
            # pixel frame to camera frame
            P_inv = np.linalg.inv(self.intrinsic_matrix)
            mat = P_inv.dot(pixel_coord)

            # camera frame to world frame
            # world frame is defined as having origin at ego vehicle's left rear wheel, z up, y forward
            cam_coord = self.T_cam_2_world
            world_coord = self.R_cam_2_world.dot(mat) + self.T_cam_2_world
            # interpolate using z coordinate
            world_coord = world_coord - (world_coord - cam_coord) / (world_coord[2, 0] - cam_coord[2, 0]) * world_coord[2, 0]

            print('Racecar Position: \n{}'.format(world_coord))
            self.racecar_pos = Point32()
            self.racecar_pos.x = float(world_coord[0, 0])
            self.racecar_pos.y = float(world_coord[1, 0])
            self.racecar_pos.z = float(world_coord[2, 0])
            self.racecar_position_publisher.publish(self.racecar_pos)

    def camera_info_callback(self, data):
        decoded = np.asarray(data.K, dtype=np.float64)
        self.intrinsic_matrix = decoded.reshape((3, 3))


def main():
    racecar_pose_estimator = RacecarPoseEstimate()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main()