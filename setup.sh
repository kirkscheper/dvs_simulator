#!/bin/bash
if [ $# -eq 0 ]
  then
    ROS_DIR=/opt/ros/kinetic
  else
    ROS_DIR=$1
fi

catkin config --init --mkdirs --extend $ROS_DIR --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

# add setup script to bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "source $DIR/devel/setup.bash" >> ~/.bashrc

#copy new scenes to simulator
cp launch/* src/rpg_davis_simulator/launch
cp datasets/scenes/* src/rpg_davis_simulator/datasets/scenes/

# set up OpenEXR
sudo apt-get install libopenexr-dev
cd src
wget http://excamera.com/files/OpenEXR-1.2.0.tar.gz -O - | tar -xz
cd OpenEXR-1.2.0
sudo python setup.py install
