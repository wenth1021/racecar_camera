#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray, Float32
from sensor_msgs.msg import Image
import time
import os


LANE_DETECTION_NODE_NAME = 'line_detection_node'
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
        frame = decodeImage(data.data, 376, 672)
        cv2.imwrite('experiment.jpg', frame)

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
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if self.min_width < w < self.max_width:
                try:
                    x, y, w, h = cv2.boundingRect(contour)
                    # img = cv2.drawContours(img.astype(np.float32), contour, -1, (0, 255, 0), 3)
                    img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                    m = cv2.moments(contour)
                    cx = int(m['m10'] / m['m00'])
                    cy = int(m['m01'] / m['m00'])
                    centers.append([cx, cy])
                    cx_list.append(cx)
                    cy_list.append(cy)
                    cv2.circle(img, (cx, cy), 7, (255, 0, 0), -1)
                    img = cv2.line(img, start_point, end_point, (0,0,0), 2)
                    img = cv2.line(img, start_point_thresh_pos, end_point_thresh_pos, (0,0,255), 2)
                    img = cv2.line(img, start_point_thresh_neg, end_point_thresh_neg, (0,0,255), 2)
                except ZeroDivisionError:
                    pass

        # Further image processing to determine optimal steering value
        try:
            if len(cx_list) >= 1:
                error_list = []
                count = 0
                for cx_pos in cx_list:
                    error = float((float(self.image_width/2) - cx_pos) / (self.image_width/2))
                    error_list.append(error)
                avg_error = (sum(error_list) / float(len(error_list)))
                p_horizon_diff = error_list[0] - error_list[-1]

                for error in error_list:
                    if abs(error) < self.error_threshold:
                        error = 0
                        error_list[count] = error
                    count += 1
                error_x = min(error_list, key=abs)
                error_x_index = error_list.index(min(error_list, key=abs))
                cy_index = cy_list.index(min(cy_list, key=abs))
                mid_x, mid_y = cx_list[cy_index], cy_list[cy_index]
                error_x = error_list[cy_index]
                print("curvy road: {}, {}".format(error_x, error_list))
        
                cv2.circle(img, (mid_x, mid_y), 7, (255, 255, 255), -1)
                start_point_error = (int(image_width/2), mid_y)
                img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
                self.centroid_error = Float32()
                self.centroid_error.data = float(error_x)
                self.centroid_error_publisher.publish(self.centroid_error)
                centers = []
                cx_list = []
                cy_list = []
        
            centers = []
            cx_list = []
            cy_list = []
            error_list = [0] * self.number_of_lines
        except ValueError:
            pass

        # try:
        #     self.centroid_error = Float32()
        #     if len(cx_list) > 1:
        #         error_list = []
        #         count = 0
        #         for cx_pos in cx_list:
        #             error = float(((self.image_width/2) - cx_pos) / (self.image_width/2))
        #             error_list.append(error)
        #         avg_error = (sum(error_list) / float(len(error_list)))
        #         p_horizon_diff = error_list[0] - error_list[-1]
        #         if abs(p_horizon_diff) <= self.error_threshold:
        #             error_x = avg_error
        #             pixel_error = int((self.image_width/2)*(1-error_x))
        #             mid_x, mid_y = pixel_error, int((image_height/2))
        #             print("straight curve: {}, {}".format(error_x, error_list))
        #         else: 
        #             for error in error_list:
        #                 if abs(error) < self.error_threshold:
        #                     error = 1
        #                     error_list[count] = error
        #                 count+=1
        #             error_x = min(error_list, key=abs)
        #             error_x_index = error_list.index(min(error_list, key=abs))
        #             mid_x, mid_y = cx_list[error_x_index], cy_list[error_x_index]
        #             print("curvy road: {}, {}".format(error_x, error_list))
                
        #         cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
        #         start_point_error = (int(image_width/2), mid_y)
        #         img = cv2.line(img, start_point_error, (mid_x, mid_y), (0,0,255), 4)
        #         self.centroid_error.data = float(error_x)
        #         self.centroid_error_publisher.publish(self.centroid_error)
        #         centers = []
        #         cx_list = []
        #         cy_list = []
        #     elif len(cx_list) == 1:
        #         mid_x, mid_y = cx_list[0], cy_list[0]
        #         error_x = float(((self.image_width/2) - mid_x) / (self.image_width/2))
        #         cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
        #         self.centroid_error.data = error_x
        #         self.centroid_error_publisher.publish(self.centroid_error)
        #         print("only detected one line")

        #     centers = []
        #     cx_list = []
        #     cy_list = []
        #     error_list = [0] * self.number_of_lines
        # except ValueError:
        #     pass


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
