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



### Lane Following and Switching Control

Associated files: **lane_detection_node.py, waypoint_follow.cpp**


### Racecar Detection with YOLO

### Racecar Position Estimation

### Overtaking Maneuver

## Instructions for Using the Package

## Demo

### Lane following

### Lane Switching

### Overtaking

### Going forward without overtaking

## Future Development and Recommendations
