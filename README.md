# Offboard Control of Pixhawk 4 using MAVSDK & ROS
## Setup
### Linux Machine (Instructions tested on Ubuntu 18.04)
To communicate with the companion computer on the drone, we use __ROS__. The instructions for installing ROS can be found here: http://wiki.ros.org/ROS/Tutorials

The relevant commands are as follows:
``` bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Congratulations, ROS is now installed. To be able to build packages:
``` bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
sudo rosdep init
rosdep update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Your workspace is now setup. The next step is to install the dependencies.
``` bash
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-dynamixel-workbench ros-melodic-dynamixel-workbench-controllers ros-melodic-dynamixel-workbench-toolbox
cd ~
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh 
sudo ./install_geographiclib_datasets.sh 

cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/mocap-optitrack
cd mocap_optitrack
git checkout new-and-old-support-updated

cd ~/catkin_ws
rm -rf devel/ build/
catkin build
```


To use this package, you will need to install the __MAVSDK C++ Library__ for your platform.
__Installation__ instructions can be found here: https://mavsdk.mavlink.io/develop/en/getting_started/installation.html

You are now ready to install this package
``` bash
cd ~/catkin_ws/src
# Use this one if you have access through gitlab
git clone https://gitlab.com/paullassen/offboard.git
# Use this one if you have access through github
https://github.com/paullassen/offboard-aut-dtu.git
catkin build
``` 


## More Stuff
If you are working on the *'autbasement'* wifi, the companion computer on the drone will have an IP-address in the form '10.59.9.xxx'. To simplify the communication between the master computer and the companion computer, we add the the names to our hosts file.

On the master computer, open /etc/hosts in an editor and add
```
10.59.9.xxx DRONE_NAME
```

Now it will be possible to ssh into the companion computer with:
```
ssh USER_NAME@DRONE_NAME
```

