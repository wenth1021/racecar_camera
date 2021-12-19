#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray, Float32
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge


LANE_DETECTION_NODE_NAME = 'lane_detection_node'
CAMERA_TOPIC_NAME = '/zed2/zed_node/rgb/image_rect_color'
CENTROID_TOPIC_NAME = '/centroid'

global mid_x, mid_y
mid_x = Int32()
mid_y = Int32()


def decodeImage(data, height, width):
    decoded = np.fromstring(data, dtype=np.uint8)
    decoded = decoded.reshape((height, width, 4))
    return decoded[:, :, :3]


class LaneDetection:
    def __init__(self):
        # Initialize node and create publishers/subscribers
        self.init_node = rospy.init_node(LANE_DETECTION_NODE_NAME, anonymous=False)
        self.camera_subscriber = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, self.locate_centroid)
        self.centroid_error_publisher = rospy.Publisher(CENTROID_TOPIC_NAME, Float32, queue_size=1)
	self.bridge = CvBridge()

        # Getting ROS parameters set from calibration Node
        # self.Hue_low = rospy.get_param('Hue_low')
        # self.Hue_high = rospy.get_param('Hue_high')
        # self.Saturation_low = rospy.get_param('Saturation_low')
        # self.Saturation_high = rospy.get_param('Saturation_high')
        # self.Value_low = rospy.get_param('Value_low')
        # self.Value_high = rospy.get_param('Value_high')
        # self.gray_lower = rospy.get_param('gray_lower')
        # self.inverted_filter = rospy.get_param('inverted_filter')
        # self.number_of_lines = rospy.get_param('number_of_lines')
        # self.error_threshold = rospy.get_param('error_threshold')
        # self.min_width = rospy.get_param('Width_min')
        # self.max_width = rospy.get_param('Width_max')
        # self.start_height = rospy.get_param('camera_start_height')
        # self.bottom_height = rospy.get_param('camera_bottom_height')
        # self.left_width = rospy.get_param('camera_left_width')
        # self.right_width = rospy.get_param('camera_right_width')
        self.Hue_low = 20 # 32
        self.Hue_high = 53 # 53
        self.Saturation_low = 100
        self.Saturation_high = 255
        self.Value_low = 0
        self.Value_high = 255
        self.inverted_filter = 0
        self.number_of_lines = 5
        self.error_threshold = 0.1
        self.min_width = 0
        self.max_width = 671
        # original width: 672
        # original height: 376
        self.start_height = 80
        self.bottom_height = 375
        self.left_width = 0
        self.right_width = 671

        # Display Parameters
        rospy.loginfo(
            '\nHue_low: {}'.format(self.Hue_low) +
            '\nHue_high: {}'.format(self.Hue_high) +
            '\nSaturation_low: {}'.format(self.Saturation_low) +
            '\nSaturation_high: {}'.format(self.Saturation_high) +
            '\nValue_low: {}'.format(self.Value_low) +
            '\nValue_high: {}'.format(self.Value_high) +
            '\ninverted_filter: {}'.format(self.inverted_filter) +
            '\nnumber_of_lines: {}'.format(self.number_of_lines) +
            '\nerror_threshold: {}'.format(self.error_threshold) +
            '\nmin_width: {}'.format(self.min_width) +
            '\nmax_width: {}'.format(self.max_width) +
            '\nstart_height: {}'.format(self.start_height) +
            '\nbottom_height: {}'.format(self.bottom_height) +
            '\nleft_width: {}'.format(self.left_width) +
            '\nright_width: {}'.format(self.right_width))

    def locate_centroid(self, data):
        # Image processing from rosparams
        #frame = decodeImage(data.data, 376, 672)
	frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
        cv2.imshow('frame', frame)

        # cropping
        self.image_width = int(self.right_width - self.left_width)
        img = frame[self.start_height:self.bottom_height, self.left_width:self.right_width]
        left_tri = np.array([(0, 0), (0, img.shape[0]-1), (int(img.shape[1] * 0.25), 0)])
        right_tri = np.array([(img.shape[1]-1, 0), (img.shape[1]-1, img.shape[0]-1), (int(img.shape[1] * 0.75), 0)])
        img = cv2.drawContours(img, [left_tri, right_tri], -1, (255,255,255), -1)

        img = cv2.GaussianBlur(img, (5,5), cv2.BORDER_DEFAULT)
        kernel_3 = np.ones((3,3), np.uint8)
        kernel_5 = np.ones((5,5), np.uint8)
        img = cv2.erode(img, kernel_3, iterations=1)
        img = cv2.dilate(img, kernel_5, iterations=1)

        cv2.imshow('cropped', img)

        image_width = self.right_width-self.left_width
        image_height = self.bottom_height-self.start_height

        # changing color space to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # setting threshold limits for white color filter
        lower = np.array([self.Hue_low, self.Saturation_low, self.Value_low])
        upper = np.array([self.Hue_high, self.Saturation_high, self.Value_high])
        mask = cv2.inRange(hsv, lower, upper)

        # creating true/false image
        if self.inverted_filter == 1:
            bitwise_mask = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask))
        else:
            bitwise_mask = cv2.bitwise_and(img, img, mask=mask)

        # changing to gray color space
        gray = cv2.cvtColor(bitwise_mask, cv2.COLOR_BGR2GRAY)

        # changing to black and white color space
        gray_lower = 25
        gray_upper = 255
        (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_lower, gray_upper, cv2.THRESH_BINARY)
        contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Setting up data arrays
        centers = []
        cx_list = []
        cy_list = []

        # Defining points of a line to be drawn for visualizing error
        start_point = (int(self.image_width/2),0)
        end_point = (int(self.image_width/2),int(self.bottom_height))

        start_point_thresh_pos_x = int((self.image_width/2)*(1-self.error_threshold))
        start_point_thresh_neg_x = int((self.image_width/2)*(1+self.error_threshold))
        
        start_point_thresh_pos = (start_point_thresh_pos_x,0)
        end_point_thresh_pos = (start_point_thresh_pos_x, int(self.bottom_height))

        start_point_thresh_neg = (start_point_thresh_neg_x,0)
        end_point_thresh_neg = (start_point_thresh_neg_x, int(self.bottom_height))

        # plotting contours and their centroids
        lines = cv2.HoughLines(blackAndWhiteImage, 1,np.pi/180,200)
	if lines is None:
		lines=[]
    	for rho,theta in lines[0]:
    	    a = np.cos(theta)
    	    b = np.sin(theta)
    	    x0 = a*rho
    	    y0 = b*rho
    	    x1 = int(x0 + 1000*(-b))
    	    y1 = int(y0 + 1000*(a))
    	    x2 = int(x0 - 1000*(-b))
    	    y2 = int(y0 - 1000*(a))

   	cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    	
    	#cv.imshow("Source", src)
    	cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", img)
    	#cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    

        # plotting results
        cv2.imshow('img', img)
        cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        cv2.waitKey(1)
        #print(mid_x, mid_y)


def main():
    lane_detector = LaneDetection()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main()
