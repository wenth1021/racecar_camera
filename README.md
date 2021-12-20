# F1TENTH Autonomous Racing using Monocular Camera

## Table of Contents

  - [**Overview**](#overview)
  - [**Setup**](#setup)
    - [Racecar Setup](#racecar_setup)
    - [Track Layout](#track_layout)
  - [**Individual Components**](#individual-components)
    - [Line Detection](#line-detection)
    - [Lane Waypoint Generation](#lane-waypoint-generation)
    - [Lane Following and Switching Control](#lane-following-and-switching-control)
    - [Racecar Detection with YOLO](#racecar-detection-with-yolo)
    - [Racecar Position Estimation](#racecar-position-estimation)
    - [Overtaking Maneuver](#overtaking-maneuver)
  - [**Demo**](#demo)
    - [Lane following](#lane-following)
    - [Lane Switching](#lane-switching)
    - [Racecar Detection and Relative Position Estimation](#racecar-detection-and-relative-position-estimation)
    - [Overtaking](#overtaking)
    - [Going forward without overtaking](#going-forward-withou-overtaking)
  - [**Future Development and Recommendations**](#future-development-and-recommendations)


## Overview

The project includes development of classic CV and learning-based perception algorithms for high-speed racing maneuvers on real F1Tenth vehicles.

Previously, F1Tenth racecar systems relied on the Lidar for navigation and did not have a vision-based navigation suite. This project develops the vision-based nav suite and employ it for situation awareness in high-speed racing scenarios. Using the vision information from a monocular camera, the package is able to make the racecar follow a designated lane using classic image processing techniques and switch lanes with either use input or racecar detection results. To detect other racecars in front of the ego racecar, we deploy a YOLO network using TensorRT, and estimate the other racecar's relative position and relative speed with respect to the ego racecar using the prior knowledge of the height. Combining the lane following and switching module and the racecar detection and position estimation module, we can make the racecar perform simple overtaking maneuver when there is another racecar right in front of the ego racecar within the same lane.

## Setup

### Racecar Setup
The racecar utilized in this project is based on the F1Tenth racecar system, utilizing the same chassis and computer, the Jetson Xavier NX. The normally equipped LiDAR however, is replaced with a forward facing camera mounted at the front of the vehicle. The specific camera used in this project is the ZED 2 - AI Stereo Camera. This camera was chosen due to a combination of factors, including camera availability and ease of use. However, the project can be easily adapted for use with any monocular camera, as the capabilities of the ZED 2 camera were limited in our use cases to 30 fps at 672x376 on a singular camera (left).

The camera is installed on a laser cut mount that screws into the chassis of the racecar. It is located at the front of the robot, positioned 3.5 inches above the top platform of the chassis and at a 22 degree downward pitch angle. The CAD files for this mount can be found [here](media/CAD).

After assembly, the camera mount looks like:
<p align="center">
<img src="media/mount2.jpg" alt="drawing" width="400"/>
</p>

The onboard Jetson Xavier NX is running on JetPack 4.6 and ROS Melodic.

### Track Layout
The track used in this project for the racecar to test in was based on a design used by races run by Jack Silberman's group at UCSD. The schematic for this track is provided [here](media/track_schematic.png).

The first iteration of the track followed closely to the schematic provided by Jack Silberman's group. However, due to physical limitations in the maximum turning radius of our F1Tenth racecars, we modified the track to eliminate the sharper turns that the racecars could not make during manual control. After adjustments, the track (shown below) is now oval shaped, with additional bends incorporated (ignore remnants of previous track in center).

![Track Layout](media/new_track.jpg)

## Individual Components

### Line Detection

Associated file: **line_detection.py**

The line detection node is used to detect the center yellow line seen on the track using classic image processing methods. After detection, the node outputs a waypoint centered on the yellow line for the racecar to follow using the waypoint following node.

The image processing steps and OpenCV methods utilized are as follows:
1. Image cropping to limit image to region of interest (lower portion of frame)
2. Gaussian Blur
3. Erosion
4. Dilation

After this step, the image looks as follows:
<p align="center">
<img src="media/cropped_frame.png" alt="drawing" width="600"/>

5. HSV Filter (Yellow)
6. Bitwise And
7. Threshold to black and white image
<p align="center">
<img src="media/blackwhite_frame.png" alt="drawing" width="600"/>
</p>

After these steps, the image frame is processed as a black and white image, in which the yellow center line markings are displayed in white and the background in black.

To prepare this black and white image to create waypoints, the OpenCV method findcontours is used to create (green) bounding contours around the lane markings. The moments of each contour is calculated and displayed as a (blue) circle on the image, representing the center of each contour. The furthest moment is chosen as the waypoint for the racecar to follow and is shown as a white circle. This allows the racecar to have a longer lookahead distance and respond better to curves and turns on the track.
<p align="center">
Straight Track
<br/>
<img src="media/line_contours_waypoint.png" alt="drawing" width="600"/>
<p align="center">
Curved Track
<br/>
<img src="media/curved_line.png" alt="drawing" width="600"/>
</p>

The distance between this chosen moment and the center line of the image (representing the heading of the vehicle) is sent to the waypoint following node for control.

Note: The HSV bounds for filtering in step 5 will need to be tuned for different cameras and different lighting conditions. Results may vary depending on the values chosen for this filter.

### Lane Waypoint Generation

Associated file: **lane_detection_node.py**

The lane detection node is used to generate waypoints centered on both lanes of the track. While the line detection node outputs one waypoint based on the center yellow line, the lane detection node creates two possible waypoints for the racecar to follow, representing the two lanes on each side of the center line. Two methods were used to generate the lane waypoints.

The first method retains the center dashed line detection from the line detection node, but also expands to detection of the solid white lane boundaries on either side of the track. Much like detection for the center yellow line, classic image processing techniques are utilized to filter out the lane boundaries and create bounding contours around them. With bounding contours of the center dashed line and the flanking lane boundaries, two waypoints are generated by finding the average lateral position between the center line and each of the boundaries, and displayed as red circles on the image.

<p align="center">
<img src="media/lane_contour.png" alt="drawing" width="600"/>
</p>

This method works well for an environment with consistent lighting conditions. However, as our track was located outdoors, the reflection of the sun off the surface of the ground would often create false positives for the white lane detection, especially in portions of the track where the racecar faces the sun. In addition, the presence of shade and shadows from surrounding structures due to the sun also affected the ability of the algorithm to correctly identify the lanes.

The second method is an extension of the line detection node and simply takes the center dashed line and extrapolates two waypoints to either side of the center line. While this method is more rudimentary, it is more robust to various lighting conditions as the yellow tape is easier to identify in our conditions than the white boundaries. Here, the lookahead distance is set at a fixed distance such that the waypoint is always shown at a fixed longitudinal position, while the lateral distance is calculated using the furthest and second furthest waypoints and a addition and subtraction by a fraction of the image width to get either side of the center line. The result is as follows:
<p align="center">
<img src="media/center_ex.png" alt="drawing" width="600"/>
</p>

### Lane Following and Switching Control

Associated files: **waypoint_follow.cpp, lane_detection_node.py**

After a waypoint is published by either the line detection or lane detection nodes to the /centroid topic, the waypoint follow node takes the waypoint and outputs an AckermannDrive message for the racecar to follow. The control algorithm is similar to that of Pure Pursuit control. Assuming that the waypoints that are published by line/lane detection are at a constant lookahead distance, the controller for steering angle is simply P control, where the input is the distance between the center of the robot and the waypoint.

As the lane detection node continuously generates two waypoints for each of the two lanes, lane switching is only a matter of selecting which waypoint for the racecar to follow. A UI element opens whenever **lane_detection_node.py** starts, allowing for manual selection of which lane for the racecar to be in. Selecting either the right or left lane will change the waypoint error that is published to the /centroid topic that the waypoint follow node uses for control.

<p align="center">
Left/Right UI Control
<br/>
<img src="media/manual_control.png" alt="drawing" width="600"/>
</p>



### Racecar Detection with YOLO
Associated files: **bbox_infer_node.cpp, bbox_infer_node.h, infer_process.cpp, infer_process.h, model_14_1.trt**

The original racecar detection package was written by Zirui Zang. We modified the code to fit our use case. The model we are currently using is a Fast YOLO network with 9 convolutional layers, and we deploy it with TensorRT to get the advantage of accelerated GPU programming. Since the model was trained using images taken indoor, it may not work reliably in the outdoor setting. Please see the following image for the pipeline of racecar detection (image courtesy of Zirui Zang).
<p align="center">
<img src="media/racecar_detection_pipeline.png" alt="drawing" width="600"/>
</p>

### Racecar Position Estimation
Associated file: **racecar_pose_estimate.py**

Estimating the 3D pose of an object using a single image from a monocular camera is not possible without any prior knowledge of the object. This is due to the scale ambiguity when we transform the coordinates from the pixel frame to the world frame. When we transform the pixel coordinates to the world frame without depth information, the resulted point can be at anywhere along the line connecting the camera origin and the correct point, as illustrated by the image below. The blue cross marks the actual detection bounding box center in the world frame, while yellow crosses are all the possible points we can get after transformation.
<p align="center">
<img src="media/racecar_pose_estimate.png" alt="drawing" width="600"/>
</p>

In our case, we assume that the two racecars are on the same leveled floor, so we can use the prior knowledge of the height of the other racecar to estimate its position on a 2D plane. We define the origin of the world frame at the left rear wheel of the ego racecar with z up and y forward, and the position of the other racecar at the center of the back of the racecar, since the center of the bounding box produced by the detection module is mostly around that point. Since we know the two cars are on the same level, the z coordinate of the other racecar should be 0 w.r.t. the world frame. We can use the z coordinate to interpolate the x and y coordinates of the other racecar on the line as shown in the image above. The relative speed along each axis can also be calculated using the timestamps, but it's not currently being used for overtaking maneuver.

The position estimation error is evaluated using the ground truth racecar position measured by hand, which can be between 0.1 to 0.3 meters assuming the bounding box produced by the detection model is reasonable. The error mostly comes from the manual extrinsic calibration to find the transformation from the camera frame to the world frame and the manual measurement of ground truth positions. We considered using the depth information from ZED camera as the ground truth; however, the depth information produced by ZED API is too noisy and unreliable to serve as ground truth. 

### Overtaking Maneuver
Associated files: **lane_detection_node.py**

The lane detection node is subscribed to the /racecar_position topic published by **racecar_pose_estimate.py**, which publishes the  position of other racecars in the camera frame relative to the ego vehicle. For static obstacles, to determine whether or not to perform a lane switch maneuver, a rectangular region is defined in front of the racecar where if any other obstacle or vehicle is detected in it, the racecar will switch lanes and overtake. To switch lanes, the algorithm chooses the other of the two waypoints identified by the lane detection module and outputs the result to the waypoint following node. For dynamic obstacles (e.g. another racecar driving on the track), the algorithm will only switch lanes if the detected obstacle is in the defined rectangular region and also has a relative speed lower than a specified threshold.

After deciding to make a lane change maneuver, there is a lockout period during which the the racecar cannot attempt to change lanes again. This is to prevent the racecar from deciding to swap lanes continuously while still in the process of moving to the other lane. In addition, after a specified amount of time after the lane change maneuver, the racecar will return to its original lane.

<p align="center">
Racecar Overtaking to Left
<br/>
<img src="media/overtake_left.png" alt="drawing" width="400"/>
</p>

In the image above, the ego racecar detects the other static racecar in front of it in the lane (represented by white circle) and decides to change to the left lane to overtake.

## Instructions for Using the Package

### Pre-requisites
1. This package is only tested on Nvidia Jetson Xavier NX board with Jetpack 4.6
2. [ZED SDK](https://www.stereolabs.com/docs/installation/jetson/), [ZED ROS wrapper](https://github.com/stereolabs/zed-ros-wrapper) (if you are also using ZED camera)

### Common Steps
1. Clone the repository into a catkin workspace and build
```shell
$ cd ~/catkin_ws/src
$ git clone https://github.com/wenth1021/racecar_camera.git
$ cd ../
$ catkin_make
$ source ./devel/setup.bash
$ find . -name “*.py” -exec chmod +x {} \;
```

2. Launch the ZED ROS nodes
```shell
$ roslaunch zed_wrapper zed2.launch
```

### To run lane following and switching only
1. Run the lane detection node
```shell
$ rosrun racecar_camera lane_detection_node.py
```
2. Run the waypoint following node
```shell
$ rosrun racecar_camera waypoint_follow
```

### To run racecar detection and position estimation only
1. Run the racecar detection node
```shell
$ rosrun racecar_camera onnx_node
```
2. Run the position estimation node
```shell
$ rosrun racecar_camera racecar_pose_estimate.py
```

## Demo
Please click the images to see the videos.
### Lane following
[![lane_following_demo](http://img.youtube.com/vi/9chVtg-OG6I/0.jpg)](http://www.youtube.com/watch?v=9chVtg-OG6I&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=2 "F1Tenth Racecar Lane Following")

### Lane Switching
[![lane_switching_demo1](http://img.youtube.com/vi/aDwI4A_th8c/0.jpg)](http://www.youtube.com/watch?v=aDwI4A_th8c&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=3 "F1Tenth Racecar Lane Switching #1")
[![lane_switching_demo2](http://img.youtube.com/vi/OUIqxRfpbgI/0.jpg)](http://www.youtube.com/watch?v=OUIqxRfpbgI&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=4 "F1Tenth Racecar Lane Switching #2")

### Racecar Detection and Relative Position Estimation
[![racecar_detection_demo1](http://img.youtube.com/vi/UgmHyZv_5KI/0.jpg)](http://www.youtube.com/watch?v=UgmHyZv_5KI&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=1 "F1Tenth Racecar Detection and Relative Position Estimation")

### Overtaking
[![overtaking_demo1](http://img.youtube.com/vi/RzejdeyGEFM/0.jpg)](http://www.youtube.com/watch?v=RzejdeyGEFM&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=5 "F1Tenth Racecar Overtaking #1")
[![overtaking_demo2](http://img.youtube.com/vi/JiLgpS6LCNQ/0.jpg)](http://www.youtube.com/watch?v=JiLgpS6LCNQ&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=6 "F1Tenth Racecar Overtaking #2")

### Going forward without overtaking
[![overtaking_demo3](http://img.youtube.com/vi/QXwajsEyNPg/0.jpg)](http://www.youtube.com/watch?v=QXwajsEyNPg&list=PLkG99R12EVl58YCTZNS6t8nJl1Y91_RCd&index=7 "F1Tenth Racecar Overtaking #3")

## Future Development and Recommendations

To further increase the speed of the racecar for lane following and switching, the control algorithm can be improved. Currently, the control of the turning angle is only a proportional control w.r.t. the error from the lane detection algorithm. To make the traveling path smoother, the control algorithm can be changed to a PID control or a model predictive control. 

To improve the reliability of the overtaking maneuver, the accuracy of the racecar pose estimation should be improved. The two major factors that can influence the accuracy of the pose estimation are the detection bounding box location and extrinsic transformation from camera frame to world frame. There is much room for the improvement for the CNN model used for racecar detection. New model structures can be used for more accurate detection such as YOLOv3 and Mask RCNN. And more data can collected in both indoor and outdoor settings to retrain the model. To obtain a more accurate extrinsic transformation, a target-based extrinsic calibration can be performed such as using AprilTag.