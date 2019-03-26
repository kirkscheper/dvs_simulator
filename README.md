## Project description
This project uses the DAVIS simulator designed by the Robotics and Perception Group from the University of Zurich to generate DVS images ('https://github.com/uzh-rpg/rpg_davis_simulator'). This repo simply adds some functionality to automatically generate image datasets from the bagfiles generated from the DAVIS simulator using various event accumulation methods.

## Dependencies
Firstly, you will need ROS to run this simulator. Here we will state the install instructions for ROS Kinetic (this is the version tested with this package), for the more recent version, please refer to the ROS wiki at [wiki.ros.org](wiki.ros.org).

### Install ROS
First add the repo
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up the keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

Update your repos
```
sudo apt-get update
```

Install ROS
sudo apt-get install ros-kinetic-ros-base

sudo apt-get install rosbag

### Install ROS required packages

Ros packages:
```
ros-kinetic-camera-info-manager
ros-kinetic-image-view
````
  
Catkin tools package
```
  python-catkin-tools
```
  
You can install all with this oneliner:
  ```sudo apt-get install ros-kinetic-camera-info-manager ros-kinetic-image-view python-catkin-tools python-rosbag
python install Pillow rospkg catkin_pkg
```

## Setup
The first time you setup the simulator you should run the setup script
  './setup.sh'
  
This will generate a catkin workspace in this directory and link to your ros installation. It defaults to ros kinetic, if you have a different version installed on your pc use
  './setup.sh PATH_TO_ROS'
  
Relacing 'PATH_TO_ROS' with your real system path to your ros installation.

It will then add the 'setup.bash' file to your bashrc so that python can find the packages that will be built later.

The script goes on to copy some new scenes and launchers to the DAVIS simulator and finally sets up OpenEXR which is needed by the DAVIS simulator.

## Building the simulator
Once you have run the 'setup.sh', you need to build the simulator with:
  './build.sh' 

## File description
- generate_bagfiles.py: generate .bagfiles with events from straight trajectories performed at constant altitude.
- generate_bagfiles_circle.py: generate .bagfiles with events from circular trajectories performed at constant altitude.
- dataset_bagfiles.py: functions needed for creating and post-processing .bagfiles.
- clean.py 
