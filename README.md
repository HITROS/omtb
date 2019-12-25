# OMTB
A modified version for OpenManipulator with TurtleBot3(OMTB)

## Environment

System: Ubuntu 16.04

ROS version: Kinetic

Gazebo version: 7.0 or higher

## Docker

Docker Container for OMTB. You can directly use the environment.

```
sudo docker pull syf758521/navigation:v1.0
```

A simple example:

```
sudo docker run -p 6080:80 -p 5900:5900 -e VNC_PASSWORD=1234 -e RESOLUTION=1920x1080 -v /dev/shm:/dev/shm syf758521/navigation:v1.0
```
Browse http://127.0.0.1:6080/

Vnc Viewer 127.0.0.1:5900

## Feature list

omtb_control

- keyboard controller for OMTB
  
- automove function for OMTB

- launch scripts for OpenManipulator
  
omtb_description

- OMTB 3D model description for visualization and simulation
  
omtb_gazebo

- simulation package using gazebo for OMTB

omtb_moveit

- Moveit controller for OpenManipulator
  
slam_lidar

- roslaunch scripts for SLAM using LiDAR

slam_vision

- roslaunch scripts for SLAM using RGB or RGBD camera

## Before started 

- You need to install some packages of ros. 

- It seems like there were some installing rules changed in 2019, some packages were deleted in ros-kinetic-desktop-full.
    
1.ros-control

```
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

2.gazebo-ros-control
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-control
```

3.ros-navigation
```
sudo apt-get install ros-kinetic-navigation
```

4.others
 ```
sudo apt-get install ros-kinetic-moveit* ros-kinetic-dynamixel-sdk ros-kinetic-dynamixel-workbench-toolbox ros-kinetic-robotis-math ros-kinetic-industrial-core ros-kinetic-smach* ros-kinetic-velodyne* ros-kinetic-hector-gazebo-plugins
 ```

## Getting started

### SLAM

1. Run environment for OMTB

```
roslaunch omtb_gazebo ${NUM}tb_room${NUM}.launch
```

Examples:

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
roslaunch slam_lidar slam.launch slam_methods:=${gmapping, hector, karto or cartographer}
```

- Using multi-turtlebot3 for SLAM (Now only support hector_SLAM)

```
roslaunch slam_lidar map_merge_hector.launch
```

3. Provide control method for OMTB

- Using keyboard

```
roslaunch omtb_control turtlebot3_key.launch
```

- Using automove function

```
roslaunch omtb_control automove_${NUM}tb_room${NUM}.launch
```

Examples:

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
roslaunch slam_lidar navigation_${NUM}tb_room${NUM}.launch
```

Examples:

- for a single turtlebot in room2

```
roslaunch omtb_control automove_1tb_room2.launch
```

- for two turtlebot3 in room2

```
roslaunch slam_lidar navigation_2tb_room2.launch
```

### Pick and place

```
roslaunch omtb_gazebo 2tb_room2_ycb.launch
roslaunch omtb_control pick_and_place_2tb_room2.launch
```

### Multi-line LiDAR

```
roslaunch omtb_gazebo 1tb_room2.launch
roslaunch slam_lidar slam.launch slam_methods:=cartographer_vlp16
roslaunch omtb_control automove_1tb_room2.launch
rosbag record /robot1/imu /robot1/odom /robot1/points2 /robot1/scan /tf /tf_static
rosservice call /robot1/write_state ${Bag_name}.pbstream
roslaunch slam_lidar assets_writer_vlp16.launch bag_filenames:=${Bag_name} pose_graph_filename:=${Bag_name}.pbstream
```
