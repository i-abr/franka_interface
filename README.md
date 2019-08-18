Franka Interface
================

This package is a middle-ware interface for the franka emika robot. The structure is borrowed
from the [franka_ros/franka_example_controllers](https://github.com/frankaemika/franka_ros) package.
The code is meant to be a stripped down version of the example controllers which takes in joint commands
from another node. The commands are filtered to avoid any time-delay or chopping behavior from the franka 
root.


Launching the nodes
-------------------

Launching the controller node and the robot state node is as easy as 
```bash
roslaunch franka_interface joint_velocity_interface.launch
```
for the joint velocity interface. Running just the robot state publisher (for HRI purposes) can be done using
```bash
robot_state.launch 
```

This package makes use of the move to start interface of the original franka_example_controllers to move
the robot using move_it package. This is accessible through
```bash
move_to_start.launch
```
