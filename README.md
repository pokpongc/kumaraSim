# KumaraSim

Gazebo simulation files and description for FIBO's Kumara Robot.

![](./Images/ExampleKumaraSim.PNG =720x)

## Prerequisite

* Ubuntu 18.04 system with ROS Melodic Morenia
* [ros_control](http://wiki.ros.org/ros_control) and [ros_controllers](http://wiki.ros.org/ros_controllers) package by:
```bash
sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
```
* MATLAB 2019b or 2020a with Simulink, [ROS Toolbox](https://www.mathworks.com/products/ros.html), and [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html) (if you want to use the provided Simulink interface) 

## Installation

Create a catkin workspace. (If you already own a catkin workspace, skip this step).
```bash
mkdir -p <WORKSPACE_NAME>/src
cd <WORKSPACE_NAME>
catkin_make
```
Now, clone this repository to your workspace.
```bash
cd src
git clone https://github.com/pokpongc/kumaraSim.git
cd ..
catkin_make
```
Select your workspace.
```bash
source <PATH_TO_WORKSPACE>/devel/setup.bash
source ~/.bashrc
```
VMware users must add `export SVGA_VGPU10=0` to `~/.profile` and then `source ~/.profile`

## Usage

To start the simulation run:
```bash
roslaunch kumara_gazebo kumara_world.launch
```
To view the head-mounted camera run:
```bash
rosrun image_view image_view image:=kumara/camera_head/image
```
Viewing the robot using Rviz:
```bash
roslaunch kumara_description kumara_rviz.launch 
```
### Simulink Interface

First, you must connect MATLAB to ROS by:
(on MATLAB command window)
```bash
rosinit('ip_address_on_ros_machine')
```
Run the Simulink model:
```bash
kumaraGazeboSim
```
To end the ROS session:
```bash
rosshutdown
```

## ROS topics
### Actuation
Actuator Related Action | ROS Topic | Message Type
------------ | ------------- | -------------
Force acted on the x_axis of the base relative to world | kumara/base_x_force_controller/command | std_msgs/Float64
Force acted on the y_axis of the base relative to world | kumara/base_y_force_controller/command | std_msgs/Float64
Torque acted on the z_axis of the base | kumara/base_z_torque_controller/command | std_msgs/Float64
X_axis position of the head relative to the body | kumara/neck_x_position_controller/command | std_msgs/Float64
Z_axis position of the head relative to the body | kumara/neck_z_position_controller/command | std_msgs/Float64
X_axis orientation of the head relative to the body | kumara/neck_rx_position_controller/command | std_msgs/Float64
Y_axis orientation of the head relative to the body | kumara/neck_ry_position_controller/command | std_msgs/Float64
Z_axis orientation of the head relative to the body | kumara/neck_rz_position_controller/command | std_msgs/Float64
Joints effort of the right arm | kumara/[q1-q5]_torque controller/command | std_msgs/Float64

### Sensing
Sensor Related Action | ROS Topic | Message Type
------------ | ------------- | -------------
Head-mounted camera's output | kumara/camera_head/image | sensor_msgs/Image
LIDAR output | kumara/laser_[position]/scan | sensor_msgs/LaserScan
SONAR output | kumara/[position]_sonar | sensor_msgs/Range
Reading joints' position and velocity | kumara/joint_states | sensor_msgs/JointState

## License
[MIT](https://choosealicense.com/licenses/mit/)
