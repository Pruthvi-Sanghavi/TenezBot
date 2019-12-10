# TenezBot :: A Tennis Ball Collecting Robot

[![Build Status](https://travis-ci.org/Pruthvi-Sanghavi/TenezBot.svg?branch=master)](https://travis-ci.org/Pruthvi-Sanghavi/TenezBot)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Coverage Status](https://coveralls.io/repos/github/Pruthvi-Sanghavi/TenezBot/badge.svg?branch=master)](https://coveralls.io/github/Pruthvi-Sanghavi/TenezBot?branch=master)

## Overview

We propose to design "TenezBot" a tennis ball collecting robot, by incorporating high-quality software engineering practices for Acme Robotics. "TenezBot" package is a complete software package which will be integrated in their new line of products. Tennis is played widely throughout the world. Mastering this game requires a good amount of time. But with that comes the demanding task of collecting hundreds of tennis balls scattered across the court, which becomes really frustrating after a tiring practice session. Thus we are aiming to develop “TenezBot” a robot which can detect ball, its position in the environment, reach out to the ball and collect it in a sac using a custom made collector attached with the TurtleBot base platform. The robot would use vision camera for vision and depth measurement. Path planning algorithms are used to reach out to the balls so that the task performance is optimum.

![Alt Text](https://github.com/Pruthvi-Sanghavi/TenezBot/blob/master/demo/demo.gif)

## Package Deliverables

This software package contains:

1. Code Files
2. Dependent Software Packages
3. World file: 
	1. Tennis Court World
	2. Empty World
4. Demo files:
	1. Bag file (Showing the simulation of robot collecting balls in the court)
	2. Visualization tools
	3. RQT Graphs
5.  Instruction manual
	- Building, Running, Tesing, Recording and Playing Instruction.
	- Hardware Implementation Instructions.

## Approach

The robot would perform the following steps to complete its task
- Sense the environment and use computer vision to detect the ball.
- Calculate the distance to the nearest Ball.
- Plan the path from the current position to the ball position coordinate.
- Reach the ball coordinate and collect the ball in sac.
- Calculate the distance to the other nearest Ball.


## Agile Iterative Process (AIP)

- [Team Review Notes](https://docs.google.com/document/d/1xD4v_xm90qUi-JRTBeB70zwtmSkE1hAyr0sW1udbbTA/edit?usp=sharing)
- [AIP Spreadsheet](https://docs.google.com/spreadsheets/d/17ZCHpeQtKOI61sXRT4afAt2YDU3qQ2JU7icFp7B_i5g/edit?usp=sharing)

## Video Demo

- Video Presentation: [link](https://drive.google.com/open?id=16ZPN-cC5ia5kIT59fHRLWBl1iRF6Wc8J)
- Presentation: [link](https://drive.google.com/open?id=155_GFJH2JAP7qsZj5m145zeNFpTQVBx8teyoY8T2POA)

## About the Developers

- Achal Vyas (linkedin:[Achal Vyas](https://www.linkedin.com/in/achal-vyas-862a43146/)): A graduate student pursuing Masters in Robotics at the University of Maryland - College Park. He is a part of Autonomous Micro Aerial Vehicles Team and has interests in Computer Vision and Machine Learning. 

- Pruthvi Sanghavi (linkedin:[Pruthvi Sanghavi](https://www.linkedin.com/in/pruthvi-sanghavi/)): A graduate student pursuing Masters in Robotics at the University of Maryland College Park. He is interested in Reinforcement Learning and Dynamics.

## Dependencies

In order to run the program you will need the following dependencies.
- Ubuntu Xenial (16.04) [click here to install](http://releases.ubuntu.com/16.04/)
- ROS Kinetic Kame [click here to install](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- OpenCV (To install follow the steps given below)
- Gazebo 7.X (Available as a part of ROS Full Desktop Version)

### Install OpenCV

- To install openCV, copy and paste the below given command in the terminal...
```
sudo apt-get update; sudo apt-get upgrade; sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev; sudo apt-get install python3.5-dev python3-numpy libtbb2 libtbb-dev; sudo apt-get install libopencv-dev libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev; libxvidcore-dev libx264-dev sphinx-common libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev; libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev; sudo apt-get install git; git clone https://github.com/opencv/opencv.git; cd opencv; mkdir build; cd build; cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH= ../ ../opencv_contrib/modules ../; make -j4; sudo make install; sudo ldconfig; sudo apt-get install python-opencv
```
To check whether OpenCV is alive and kicking, type as given below.
```
python
import cv2
print(cv2.__version__)
``` After the version gets printed, exit the python command line by typing ```
exit()
```

## ROS Dependencies
CV_Bridge

## Creating Workspace and Package

Create a ROS Workspace by running the following commands. (Replace <workspace> with the workspace name of your liking.)

- Open a terminal
```
mkdir -p ~/<workspace>/src
cd <workspace>
catkin_make
source devel/setup.bash
``` 
For creating the TenezBot Package, Run the following commands. 
```
cd src
git clone https://github.com/Pruthvi-Sanghavi/TenezBot
cd ..
catkin_make
```

## Running the Demo

To run the demo, run the following commands in the terminal.

- Open a terminal

```
cd <workspace>
source devel/setup.bash
roslaunch tenezbot tenezbot.launch
```

## Bag file

### Recording the Bag file
- A ros bag file records the topic and messages being published in the terminal. Run the following commands in the terminal.

```
cd <workspace>
source devel/setup.bash
roslaunch tenezbot tenezbot.launch record:=enable

```
The bag file is saved as tenezbot.bag in the results directory.

### Inspecting the bag file

To get the information about the bag file, run the following the terminal.
Open a terminal

```
cd <workspace>/src/tenezbot/results
rosbag info tenezbot.bag

```

### Playing the bag file

In order to play a bag file, run the following commands in the terminal:

Open a terminal

```
roscore
```
Open a new terminal

```
cd <workspace>/src/tenezbot/results/
rosbag play tenezbot.bag
```

## Run Tests
```
cd <workspace>
catkin_make run_tests	

```

## Plugins

##### CppChEclipse

If cppcheck package is not installed, then to install the package, run the following command:
```
sudo apt-get install cppcheck
```
To run cppcheck in Terminal, run the following commands:
```
cd ~/<workspace>/src/TenezBot
cppcheck --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```

##### Google C++ Style

If cpplint package is not installed, then to install the package, run the following command:
```
sudo apt-get install python-pip
pip install cpplint
```
To check Google C++ Style formatting in Terminal, run the following commands:
```
cd ~/<workspace>/src/TenezBot
cpplint $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```


## Generating Doxygen Documentation

To install doxygen run the following command:
```
sudo apt install doxygen
```
<!-- 
cd ~/<workspace>/src/TenezBot
mkdir docs
doxygen -g config
```
Open the Doxygen configuration file "config" and update the following parameters:

* PROJECT_NAME = "object_collection_robotic_arm"

* INPUT = ./src ./include/ ./test

* OUTPUT_DIRECTORY = docs

Then, rename the "config" file to "doxconfig".
 -->

Now, to generate doxygen documentation, run the following commands:

```
cd ~/catkin_ws/src/TenezBot/
doxygen -g
doxygen doxconfig
```
Doxygen files will be generated to /docs folder. To view them in a browser, run the following commands:

```
cd docs/html
firefox index.html
```
## Known Issues/Bugs

- Ball color is fixed.
- Ball collection requires a speacial mechanism. It can be demonstrated by making the balls to disappear using ros service.
- Robot has a limited depth and vision.



## License 


```
 BSD 3-Clause License

Copyright (c) 2019, Pruthvikumar Sanghavi, Achal Vyas.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```




