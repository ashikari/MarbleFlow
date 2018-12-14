# MarbleFlow
Optical marble tracker using a mobile camera

## Goal
We completed two main functional goals. First, we build a ball tracking system that used threshold based object detection to locate the 2D centroid position of a ball in a video feed from a camera mounted on a Sawyer robot arm. The centroid position was subtracted from the image center position to provide an error signal in x, and y. This error signal was used as the reference signal for a velocity controller on Sawyer, thus enabling the robot arm to follow a ball around based on the camera input. The second algorithm we implemented was dense optical flow which we implemented on the full image. Taking an average of the dense optical flow gave velocity estimates of the camera motion which allowed us to recreate the estimated trajectory of the camera based purely on the video feed. To improve our results, we printed a checked background for the balls to travel over. The background was filtered out of the image before the object tracker was run to prevent erroneous detection.

## Methods
![Algorithm Block Diagram](/block_diagram.png?raw=true)
TODO: Abbas

## Hardware
Component | Selected Hardware Implementation
--------- | --------------------------------
Computer | Dell Precision 7720
Camera | Intel RealSense Depth Camera D435
Robot Arm | RethinkRobotics Sawyer

## Software
With the exception of the proprietary controller software integrated into the Sawyer contol computer, all software was tested using a Dell Precision 7720 running Ubuntu 16.04. The camera video feed was processed in C++ using OpenCV 3.3.1. As illustreated in the Methods section, the C++ code produced a position error vector from the ball to the center of the image. This error vector was communicated over the ROS framework as an Int32MultiArray to a short Python script which computes a basic PID velocity control effort from the position error vector and commands the Sawyer to move at the calcualted velocity using the RethinkRobotics Intera SDK.

## Results
[![Results](https://img.youtube.com/vi/sgq_YhP6NFc/0.jpg)](https://www.youtube.com/watch?v=sgq_YhP6NFc)

## Future Work
TODO: Abbas

## Installation Guide

### Install Dependencies
Run all of the following in Terminal.

#### ROS
The following is adapted from http://sdk.rethinkrobotics.com/intera/Workstation_Setup.
```
$ sudo sh -c ​ 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers ros-kinetic-xacro ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-actionlib ros-kinetic-actionlib-msgs ros-kinetic-dynamic-reconfigure ros-kinetic-trajectory-msgs ros-kinetic-rospy-message-converter
```

#### OpenCV
The following is adapted from https://www.learnopencv.com/install-opencv3-on-ubuntu/.
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get remove x264 libx264-dev
$ sudo apt-get install build-essential checkinstall cmake pkg-config yasm
$ sudo apt-get install git gfortran
$ sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
$ sudo apt-get install libtiff5-dev
$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
$ sudo apt-get install libxine2-dev libv4l-dev
$ sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
$ sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
$ sudo apt-get install libatlas-base-dev
$ sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
$ sudo apt-get install libvorbis-dev libxvidcore-dev
$ sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
$ sudo apt-get install x264 v4l-utils
$ git clone https://github.com/opencv/opencv.git
$ cd opencv 
$ git checkout 3.3.1 
$ cd ..
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd opencv_contrib
$ git checkout 3.3.1
$ cd ..
$ cd opencv
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
$ make
$ sudo make install
$ sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
$ sudo ldconfig
```

#### RealSense Camera
TODO: Abbas

### Installation
Run the following commands in Terminal in the directory in which you would like to keep MarbleFlow.
```
$ git clone https://github.com/ashikari/MarbleFlow.git
$ cd MarbleFlow/src
$ wstool init .
$ git clone https://github.com/RethinkRobotics/sawyer_robot.git
$ wstool merge sawyer_robot/sawyer_robot.rosinstall
$ wstool update
$ cd ..
$ catkin_make
$ cd src
$ git clone https://github.com/rupumped/sawyer_pykdl.git
$ cd ..
$ catkin_make.
```

In addition to cloning the Intera repositories into the repository, you will also need to copy and customize the `intera.sh` script. To do so, enter the following commands into Terminal in the MarbleFlow directory. The following is adapted from http://sdk.rethinkrobotics.com/intera/Workstation_Setup.

First, copy the `intera.sh` script.
```
$ cp src/intera_sdk/intera.sh intera.sh
```

Now, open the script in your favorite text editor, for example:
```
$ gedit intera.sh
```
Your robot's hostname is defaulted as the ​ Controller's Serial Number​​ and NOT the Robot's Serial Number. The serial number can be located on the back of the robot's controller box. Unless you intend to modify the default​ ​ Networking​ configuration, leave the ".local" suffix on the end of the Controller's Serial Number in the ​robot_hostname.local​​ field.

Comment where 'your_ip' is the IP address of your PC.
```
# your_ip="192.168.XXX.XXX"
```

Uncomment and modify where 'your_hostname' is your computer’s hostname, followed by ".local". Alternatively, you can use the bash shortcut, ​`uname`.
```
your_hostname=​ "$(uname -n).local"
```

Verify that the the 'ros_version' field matches the ROS version you are running: This field will default to "indigo". To use another ROS version, update:
```
ros_version=​ "kinetic"
```

Save and Close intera.sh script.

### Testing Installation
Power on and connect your Sawyer and camera. Then run the following commands in Terminal in the MarbleFlow directory:
```
$ ./intera.sh
$ rosrun marble_flow sawyer_comm.py
```
The Sawyer robot arm should move to the starting position.

Then, open a new Terminal to the same directory and enter:
```
$ ./intera.sh
$ rosrun marble_flow blob_error_pub
```
A window should appear on screen containing the video feed from the camera. If a marble is detected, a filled red circle will appear over the marble with an arrow pointing to the center of the screen. The Sawyer should begin moving to reduce this error.

## Repository Overview
```
.
|
+-- src
|   +-- marble_flow
|       +-- src
|           +-- blob_error_pub.cpp    Reads camera video feed and calculates and publishes position error.
|           +-- blob_tracker.hpp      Header file for high level CV algorithms.
|           +-- go_to.py              Command motion to specific joint angles for Sawyer.
|           +-- sawyer_comm.py        Receives position error from C++ and commands Sawyer to move.
|
+-- test                              Testing implementations of OpenCV algorithms in C++.
|   +-- DISP_RW                       Scripts to read and write image and video data.
|       +-- disp_cam.cpp              Read and display video data from webcam in real time.
|       +-- disp_pic.cpp              Read and display image from file.
|       +-- disp_vid.cpp              Read and display video from file.
|       +-- pixel_write.cpp           Draw line on image from file.
|   +-- OLDCV_code/                   OpenCV2 code.
|   +-- realsense                     Testing the RealSense camera.
|       +-- open_cam.cpp              Read and dispaly video data from RealSense camera.
|   +-- transforms                    Common image transformations.
|       +-- canny.cpp                 Canny edge detection on image from file.
|       +-- canny_cam.cpp             Canny edge detection on video data from camera.
|       +-- erode.cpp                 Erosion on video data from camera.
|       +-- simple_transform.cpp      Gaussian blur on image from file.
|   +-- blank.cpp                     Draw square.
|   +-- flow.cpp                      Optical flow.
|   +-- flow.hpp                      Optical flow header file.
|   +-- movie_maker.hpp               Header file to publish video data to file.
|   +-- plot_flow.cpp                 Use optical flow to estimate and plot position.
```
