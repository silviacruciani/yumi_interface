# yumi_interface

This ros package contains a python interface to an arm the yumi robot using the velocity control mode.
<br />
It uses this [yumi](https://github.com/kth-ros-pkg/yumi) package to communicate with the robot or the simulation.
<br />
It requires PyKDL.

The joints are stored in the order of the kinematic chain, but it does not change the naming convention.
The inputs can follow this ordering or the default numbering, and a bool variable indicates what convention to follow when a method is called.

- *Kinematic chain order*: [1, 2, 7, 3, 4, 5, 6] 

- *redefine_idx* set to True indicates that the index is one of the names, not the index in the kinematic chain order.

### useful methods: 

- set_joint_positions(positions, kinematic_chain_order = {True, False}): sets the joints to the desired positions
- set_joint_position(idx, position, redefine_idx = {True, False}): sets the idx-th joint to the desired position
- set_joint_velocities(velocities, kinematic_chain_order = {True, False}): sets the desired joint velocities
- set_joint_velocity(idx, velocity, redefine_idx = {True, False}): sets the desired velocity for idx-th joint to the desired position
- move_ee_to_pose(pose): moves the end-effector to the desired pose
- set_ee_velocity(velocity): sets the desired cartesian velocity for the end-effector
- get_forward_kinematics(): returns the current pose of the end-effector

### Actionlib

The push action approaches a desired point and pushes on the given direction. (Rotate action is under development).
<br />
Inputs: 
- target frame (tf) 
- arm to use (left, right)
- push direction (3d vector)
- approach distance before pushing

