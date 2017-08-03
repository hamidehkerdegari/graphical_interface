# Graphical Interface

## Installation:

#### ROS Installation:

```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu
$(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop ros-kinetic-image-transport-plugins
```

#### Configure installation for use with MIRO:

Add the following lines to the file ~/.bashrc, as preferred, on your workstation.

```sh
# configuration
export MIRO_PATH_MDK=~/mdk
export ROS_IP=193.168.0.3
export ROS_MASTER_URI=http://localhost:11311

# usual ROS setup
source /opt/ros/kinetic/setup.bash

# make our custom messages available to ROS/python
export ROS_PACKAGE_PATH=$MIRO_PATH_MDK/share:$ROS_PACKAGE_PATH
export PYTHONPATH=$MIRO_PATH_MDK/share:$PYTHONPATH
```


#### Install Gazebo:

```sh
$ sudo apt-get install gazebo7 libgazebo7-dev
```
