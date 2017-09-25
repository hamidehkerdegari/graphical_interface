# Graphical Interface

## User Instructions
Follow instructions below in order to run the MiRo Graphical Interface on your Ubuntu system:

###Installation:
#### 1. Installing ROS:
To run the graphical interface, the first step is to install the ROS. Detailed instructions are provided in (at least the ROS core needs to be installed):

http://wiki.ros.org/ROS/Installation

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

You can modify the ~/.bashrc file by:
```sh
sudo nano ~/.bashrc
```
Copy and past the lines above and pres ctrl+x and then confirm the modifications by pressing y.

#### 3. Gazebo installation:
Although this is not required, but you can install the Gazebo simulator by:
```sh
$ sudo apt-get install gazebo7 libgazebo7-dev
```

Then configure installation for use with MIRO by adding the following lines to the file ~/.bashrc, as preferred, on your workstation.

```sh
# usual Gazebo setup
source /usr/share/gazebo/setup.sh

# announce MIRO resources to Gazebo
export GAZEBO_RESOURCE_PATH=$MIRO_PATH_MDK/share:${GAZEBO_RESOURCE_PATH}
```

#### 4. Setting up the P3 on your MiRo:
 
 [MIRO: Maintenance](https://consequential.bitbucket.io/Technical_Processors_Maintenance.html#Reprogram%20P3)

1. Format the SD card
2. Run:
```sh
$ cd ~/mdk/bin/deb64
$ sudo ./program_P3.sh /dev/sdb --pass=1234 --network=SSID/PASSWORD --masteraddr=193.168.0.100
```

#### 5. Setting up P2:

```sh
$ ssh 193.168.0.2
```

```sh
$ cd ~/mdk/bin/am335x
$ ./program_P2.sh main
```
 
#### 6. Install latest MDK version on your root. 


#### 7. Downloading the Graphical Interface:

If Git is not installed, you first need to install it by:
```sh
sudo apt-get install git
```

The most current version of the graphical interface can be downloaded by typing into the terminal:
```sh
cd
git clone https://github.com/hamidehkerdegari/graphical_interface.git --origin upstream
```

To get updates and bug fixes, you can go to the graphical interface directory,
```sh
cd ~/graphical_interface
```

and type in:
```sh
git pull
```

If "git pull" prints out a message telling it cannot pull the remote changes because you have changed files locally, you may have to commit locally and merge your changes, or stash them temporarily and then apply back the stash; for that, we recommend that you read about how Git works.


### Running the graphical interface:
Now everything is installed. So open a new terminal and run:
```sh
roscore
```
Open a second terminal and run the following to connect to the MiRo with the address of 193.168.0.1:
```sh
ssh root@193.168.0.1
```

Enter MiRo password:
```sh
1234
root@miro:~>
```

Then run:
```sh
run_bridge_ros.sh
```

Open a third terminal and run the following main code for the graphical interface:
```sh
cd ~/graphical_interface/ui/
./gi_run.py robot=rob01
```

After running all the above commands, the graphical interface will be visualized on the desktop.

### Further information:
You can use some shortcuts on the graphical interface:

- Ctrl+F: For full screening the interface
- Ctrl+W: For closing the interface
- Ctrl+S: For saving the interface as a picture
