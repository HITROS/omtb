/*******************************************************************************
* Copyright 2019 HITROS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Yu Fu */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/KinematicsPose.h>
#include <open_manipulator_msgs/JointPosition.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <Eigen/Eigen>
#include <omtb_control/omtb_kinematics.h>

using namespace std;


geometry_msgs::Pose get_pose(float position[])
{
  geometry_msgs::Pose object_pose;
  object_pose.position.x = position[0];
  object_pose.position.y = position[1];
  object_pose.position.z = position[2];
  float dist = sqrt((object_pose.position.x * object_pose.position.x) +
                   (object_pose.position.y * object_pose.position.y));
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  if (object_pose.position.y > 0)
      yaw = acos(object_pose.position.x / dist);
  else
      yaw = (-1) * acos(object_pose.position.x / dist);
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  object_pose.orientation.w = cy * cr * cp + sy * sr * sp;
  object_pose.orientation.x = cy * sr * cp - sy * cr * sp;
  object_pose.orientation.y = cy * cr * sp + sy * sr * cp;
  object_pose.orientation.z = sy * cr * cp - cy * sr * sp;
  return object_pose;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "omtb_moveit");
  ros::NodeHandle node_handle("");
//  std::string  planning_group = argv[1];
//  std::string service_name;
//  service_name << planning_group << '/moveit/set_kinematics_pose';

  ros::ServiceClient arm_client = node_handle.serviceClient<open_manipulator_msgs::SetJointPosition>("robot1/arm/moveit/set_joint_position");
  ros::ServiceClient gripper_client = node_handle.serviceClient<open_manipulator_msgs::SetJointPosition>("robot1/gripper");

  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("robot1/odom", "robot1/link1", ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("robot1/odom", "robot1/link1", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  Eigen::Quaterniond link1_to_world_q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
  Eigen::Isometry3d link1_to_world(link1_to_world_q);
  link1_to_world.pretranslate( Eigen::Vector3d( transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));
  Eigen::Isometry3d end_to_world=Eigen::Isometry3d::Identity();
  float position1[3] = {0.161, 0.1, 0.200};
  end_to_world.pretranslate(Eigen::Vector3d(position1[0], position1[1], position1[2]));
  Eigen::Isometry3d end_to_link1 = end_to_world*link1_to_world.inverse();
  Eigen::Vector3d goal_to_link1_vector = end_to_link1.translation();
  std::vector<double> goal_vector;
  goal_vector.push_back(goal_to_link1_vector(0));
  goal_vector.push_back(goal_to_link1_vector(1));
  goal_vector.push_back(goal_to_link1_vector(2));
  OmtbKinematics solver;
  if (solver.inverse_kinematics(goal_vector))
  {
    open_manipulator_msgs::SetJointPosition arm_test;
  //  float position1[4] = {0.1, 0.5, 0.1 -0.6};

    open_manipulator_msgs::JointPosition arm_joint;
    arm_joint.position = solver.joint_position_;
    arm_joint.max_velocity_scaling_factor = 0.1;
    arm_joint.max_accelerations_scaling_factor = 0.1;

    arm_test.request.joint_position = arm_joint;
    arm_test.request.planning_group = "arm";

    if (arm_client.call(arm_test)) {
        if (arm_test.response.is_planned)
        {
          cout<<"success"<<endl;
        }
        else
        {
          cout<<"fail"<<endl;
        }
    }
    else {
        ROS_ERROR("Failed to call service!!");
        return 1;
    }
  }
  ros::Duration(2.0).sleep();
  open_manipulator_msgs::SetJointPosition gripper_test;
  open_manipulator_msgs::JointPosition gripper_joint;
  gripper_joint.position = {0.15};
  gripper_joint.max_velocity_scaling_factor = 0.1;
  gripper_joint.max_accelerations_scaling_factor = 0.1;
  gripper_test.request.planning_group = "arm";
  gripper_test.request.joint_position = gripper_joint;
  if (gripper_client.call(gripper_test)) {
      if (gripper_test.response.is_planned)
      {
        cout<<"success"<<endl;
      }
      else
      {
        cout<<"fail"<<endl;
      }
  }
  else {
      ROS_ERROR("Failed to call service!!");
      return 1;
  }
}
