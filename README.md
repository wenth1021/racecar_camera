# F1TENTH Autonomous Racing using Monocular Camera

## Table of Contents

  - [**Overview**](#overview)
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


## Individual Components

### Line Detection

### Lane Waypoint Generation

### Lane Following and Switching Control 

### Racecar Detection with YOLO

### Racecar Position Estimation

### Overtaking Maneuver

## Instructions for Using the Package

## Demo
Click the images to see the videos.
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
