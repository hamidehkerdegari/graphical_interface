# Graphical Interface

## Installation:

#### 1. ROS Installation:

```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu
$(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop ros-kinetic-image-transport-plugins
```

#### 2. Configure installation for use with MIRO:

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


#### 3. Gazebo installation:

```sh
$ sudo apt-get install gazebo7 libgazebo7-dev
```

Configure installation for use with MIRO:

Add the following lines to the file ~/.bashrc, as preferred, on your workstation.

```sh
# usual Gazebo setup
source /usr/share/gazebo/setup.sh

# announce MIRO resources to Gazebo
export GAZEBO_RESOURCE_PATH=$MIRO_PATH_MDK/share:${GAZEBO_RESOURCE_PATH}
```

#### 4. Setting up the P3:
 
 [MIRO: Maintenance](https://consequential.bitbucket.io/Technical_Processors_Maintenance.html#Reprogram%20P3)

1. Format the SD card
2. Run:
```sh
$ cd ~/mdk/bin/deb64
$ sudo ./program_P3.sh /dev/sdb --pass=1234 --network=HamidehAP/95451695 --masteraddr=193.168.0.3
```

#### 5. Setting up P2:

```sh
$ ssh 193.168.0.2
```

```sh
$ cd ~/mdk/bin/am335x
$ ./program_P2.sh main
```

---
