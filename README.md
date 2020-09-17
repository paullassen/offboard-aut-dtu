# Offboard Control of Pixhawk 4 using MAVSDK & ROS
## Dependencies
To use this package, you will need to install the __MAVSDK C++ Library__ for your platform.
__Installation__ instructions can be found here: https://mavsdk.mavlink.io/develop/en/getting_started/installation.html
Instruction for __Building__ from source can be found here: https://mavsdk.mavlink.io/develop/en/contributing/build.html

This package also requires __ROS__. 
Installation instructions can be found here: http://wiki.ros.org/ROS/Installation

If you are using __OptiTrack__ motion capture system, it is recommended to use __Motive__ version __1.7__. 
The corresponding ros package can be found here:
https://github.com/ros-drivers/mocap_optitrack/tree/new-and-old-support-updated
To install the correct version:
```bash
git clone https://github.com/ros-drivers/mocap_optitrack.git
git checkout new-and-old-support-updated
```
## Setup
If you are working on the *'autbasement'* wifi, the companion computer on the drone will have an IP-address in the form '10.59.9.xxx'. To simplify the communication between the master computer and the companion computer, we add the the names to our hosts file. 

On the master computer, open /etc/hosts in an editor and add
```
10.59.9.xxx DRONE_NAME
```

Now it will be possible to ssh into the companion computer with:
```
ssh USER_NAME@DRONE_NAME
```

