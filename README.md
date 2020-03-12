
# ROS-Quadcopter-Simulation
This is an on-going project on simulating drone and stabilizing it using Approximate Dynamic Programming on ROS-Gazebo (melodic version of ROS and Gazebo version 9)

This is a simulation of Joop-Brokking's Quadcopter. He has great tutorial videos on YouTube:

YouTube Channel: https://www.youtube.com/channel/UCpJ5uKSLxP84TXQtwiRNm1g

Website: http://www.brokking.net/


The Model of the Quad is written in .xacro format, and is imported in Gazebo simulator.

All the mass, moment of inertia etc are identical to the DJI-f450 frame. Further, the libLiftDragPlugin and ROS_control plugins have been used, to provide lift due to the rotating propellers, and providing accurate velocities to the four BLDC motors respectively.

Any progress/add-ons will be updated in this repository.

![Image](https://github.com/NishanthARao/ROS-Quadcopter-Simulation/blob/master/Image.png)

# Installation #

Make sure you've installed ROS and Gazebo on your systems.

Additionally, you have to install the following packages:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 
sudo apt-get update
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-control ros-melodic-ros-controllers
sudo apt-get install ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-position-controllers
```
If you have kinetic version, please follow the instructions of ros-control here:

http://wiki.ros.org/ros_control

Open a terminal.
1. Initiate a workspace in your home directory or use your existing favorite one.
```
source /opt/ros/melodic/setup.bash 

mkdir -p ~/danger_ws/src
cd ~/danger_ws/src
catkin_init_workspace
```

2.Create necessary workspace files
```
cd ~/danger_ws
catkin_make
```

3.Add this workspace to your linux environment by sourcing the setup file to .bashrc. Assuming you are inside the home directory, 
```
cd ~
gedit .bashrc
```
Add this line at the end of the file.
```
source ~/danger_ws/devel/setup.bash
```

4.Create a ROS package in your workspace. We will call it fly_bot. Add the rospy and std_msgs dependencies
```
cd ~/danger_ws/src
catkin_create_pkg fly_bot rospy std_msgs
```

5.Download all the folders and files into the folder fly_bot. i.e all the folders and files seen in this repo must be present inside the fly_bot. Donot create another folder inside the fly_bot with all theses files.

Note: You have to replace the existing src folder and CMakeLists and package files with this repo's folder and files. 
The folder hierarchy thus, must be:
```
danger_ws/src/fly_bot
  -/config
  -/launch
  -/meshes
  -/src
  .
  .
  .
  -CMakeLists.txt
  -package.xml
  -urdf.rviz
```

Then,
```
cd ~/danger_ws/src/fly_bot/src
chmod u+x control.py
chmod u+x pid.py
```

6.Execute the following command to build into your ROS workspace
```
cd ~/danger_ws
catkin_make
```

This should build the directory without any errors. If you find any errors, please check your steps with those mentioned here.

Once installed, close the terminal. Open another terminal and load the quadcopter into gazebo simulator
```
roslaunch fly_bot Kwad_gazebo.launch
```

This should load the Quadcopter into Gazebo simulator. You may get some errors of sort "No p gains mentioned in pid....", "Bad callback; IndexError: Out of index" or "No name 'Kwad' found" - and that's fine.

For the quadCopter to just hover in mid-air, open another terminal and type in the following command
```
rosrun fly_bot control.py
```

You should see the Quadcopter fly upwards while stabilizing itself.

Alternatively, you can provide commands to individual motors (here, there is no PID control and stabilization of the Quad):
```
rostopic pub -1 /Kwad/joint_motor_controller/command std_msgs/Float64MultiArray "data: [50, -50, 50, -50]"
```
This provides a speed of:

i)   50 units to front_right motor

ii) -50 units to front_left motor

iii) 50 units to back_left motor

iv) -50 units to back_right motor

Here, the negative sign denotes rotation in the opposite direction.


The pid values are in the /src/pid.py file in your fly_bot directory. Play around with the values to see some control theory in action!
