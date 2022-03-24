# Installation Instructions and required libraries

In order to install this repository, follow the steps outlined below. The installation has been tested and is only functional in Ubuntu 20.04

## Install and configure GitHub LFS

[GitHub LFS](https://git-lfs.github.com/ 'Install GitHub LFS')

Follow the instructions under Getting Started, first:

`sudo ./install`
	
Then:
	
`git lfs install`
	
No need to add anything if only downloading files
	
### If you add a heavy (>100Mb) file in the repository

- "delete" file (move out of repository)
- git lfs track "filename" (without the specific directory within the repository, only file name)
- git add .gitattributes
- commit
- push
- put file
- push again
	
## Install ROS Noetic

[ROS Noetic Installation](http://wiki.ros.org/noetic/Installation 'Install ROS Noetic')

## Install Octomap Libraries

`sudo apt-get install ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-msgs ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-octomap-server`

## Install ros control

`sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

## Install and update pip

`sudo apt install python3-pip`

`pip install --upgrade pip`

## Install OpenCV

`sudo apt install libgl1-mesa-glx ffmpeg libsm6 libxext6`

`pip install opencv-contrib-python`

## Install TensorFlow

[TensorFlow installation](https://www.tensorflow.org/install 'Install TensorFlow')

To install you don't need the website, only need to do:

`pip install tensorflow`

If there is no tensorflow version available (e.g. if you use a Raspberry Py) you may have to install from source:

[TensorFlow installation from source](https://www.tensorflow.org/install/source 'Install TensorFlow from source')

## Install ompl library

`sudo apt install libompl-dev`

## Install FCL library

First install boost library

`sudo apt-get install libboost-all-dev`

Then install libccd

`sudo apt-get install libccd-dev`

Finally install the library

Follow the instructions on its README and USE VERSION 0.5

[Flexible Collision Library](https://github.com/flexible-collision-library/fcl/releases 'Install FCL')
