# Kumara Sim

Gazebo simulation files and description for FIBO's Kumara Robot.

### CAD
- [x] Base
- [ ] 5 DOF arm
- [ ] Hand
- [ ] Body

## Prerequisite

* Ubuntu 18.04 system with ROS Melodic Morenia
* [ros_control](http://wiki.ros.org/ros_control) package
* MATLAB 2019b or 2020a with Simulink, [ROS Toolbox](https://www.mathworks.com/products/ros.html), and [Robotics System Toolbox](https://www.mathworks.com/products/robotics.html) (if you want to use the provided Simulink interface) 

## Installation

If you already own a catkin workspace, skip this step. Create a catkin workspace.
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
Control parameter | ROS topic
------------ | -------------
Force acted on the x_axis of the base relative to world | kumara/base_x_force_controller/command
Force acted on the y_axis of the base relative to world | kumara/base_y_force_controller/command
Torque acted on the z_axis of the base | kumara/base_z_torque_controller/command
X_axis position of the head relative to the body | kumara/neck_x_position_controller/command
Z_axis position of the head relative to the body | kumara/neck_z_position_controller/command
X_axis orientation of the head relative to the body | kumara/neck_rx_position_controller/command
Y_axis orientation of the head relative to the body | kumara/neck_ry_position_controller/command
Z_axis orientation of the head relative to the body | kumara/neck_rz_position_controller/command
Joints effort of the right arm | kumara/[q1-q5]_torque controller/command
Head-mounted camera's output | kumara/camera_head/image
Reading joints' position and velocity | kumara/joint_states

## License
[MIT](https://choosealicense.com/licenses/mit/)
