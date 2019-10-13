# OMTB
A modified version for OpenManipulator with TurtleBot3(OMTB)

## Environment

System: Ubuntu 16.04

ROS version: Kinetic

Gazebo version: 7.0 or higher

## Feature list

omtb_control

- provides controller using keyboard for OMTB
  
- provides automove function for OMTB
  
omtb_description

- provides OMTB 3D model description for visualization and simulation
  
omtb_gazebo

- includes simulation package using gazebo for OMTB
  
omtb_slam2d

- provides roslaunch scripts for starting the 2D SLAM

## Getting started

1. Run environment for OMTB

```
Template:
roslaunch omtb_gazebo ${NUM}tb_room${NUM}.launch
```

Example:

- A single turtlebot in room1

```
roslaunch omtb_gazebo 1tb_room1.launch
```
- Two turtlebots in room2

```
roslaunch omtb_gazebo 2tb_room2.launch
```

- Two turtlebots in room2 with ramp1

```
roslaunch omtb_gazebo 2tb_room2_ramp1.launch
```

2. Launch slam

- Using single-turtlebot3 for SLAM

```
Template:
roslaunch omtb_slam2d omtb_slam.launch slam_methods:=${gmapping, hector, karto or cartographer}
```

- Using multi-turtlebot3 for SLAM (Now only support hector_SLAM)

```
roslaunch omtb_slam2d map_merge_hector.launch
```

3. Provide control method for OMTB

- Using keyboard

```
roslaunch omtb_control turtlebot3_key.launch
```

- Using automove function

```
Template:
roslaunch omtb_control automove_${NUM}tb_room${NUM}.launch
```

Example:

- for a single turtlebot3 in room1

```
roslaunch omtb_control automove_1tb_room1.launch
```

- for two turtlebot3 in room2

```
roslaunch omtb_control automove_1tb_room2.launch
```

4. Save map

```
Template:
rosrun map_server map_saver map:=/robot${NUM}/map -f /${YOUR PATH}
```

5. Start robot navigation

```
Template:
roslaunch omtb_slam2d navigation_${NUM}tb_room${NUM}.launch
```

Example:

- for a single turtlebot in room2

```
roslaunch omtb_control automove_1tb_room2.launch
```

- for two turtlebot3 in room2

```
roslaunch omtb_slam2d navigation_2tb_room2.launch
```
