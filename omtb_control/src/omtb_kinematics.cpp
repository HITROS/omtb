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

#include <omtb_control/omtb_kinematics.h>

#define PI 3.1415926
using namespace std;

const double l1 = 0.012;
const double l2 = 0.017;
const double l3 = 0.058;
const double l4 = 0.024;
const double l5 = 0.128;
const double l6 = 0.124;
const double l7 = 0.126;
const double alpha = atan2(l5,l4);
const double error = 0.00001;


bool OmtbKinematics::forward_kinematics(std::vector<double> joint_position)
{
  double x1 = 0;
  double y1 = 0;
  double z1 = 0;

  double x2 = l1;
  double y2 = 0;
  double z2 = l2;

  double x3 = l1;
  double y3 = 0;
  double z3 = l2 + l3;

  double x4 = l1 + (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1)))*cos(joint_position.at(0));
  double y4 = (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1)))*sin(joint_position.at(0));
  double z4 = l2 + l3 + sqrt(l4*l4+l5*l5)*sin(alpha-joint_position.at(1));

  double x5 = l1 + (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1))+l6*cos(joint_position.at(1)+joint_position.at(2)))*cos(joint_position.at(0));
  double y5 = (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1))+l6*cos(joint_position.at(1)+joint_position.at(2)))*sin(joint_position.at(0));
  double z5 = l2 + l3 + sqrt(l4*l4+l5*l5)*sin(alpha-joint_position.at(1))-l6*sin(joint_position.at(1)+joint_position.at(2));

  double x6 = l1 + (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1))+l6*cos(joint_position.at(1)+joint_position.at(2))+ l7)*cos(joint_position.at(0));
  double y6 = (sqrt(l4*l4+l5*l5)*cos(alpha-joint_position.at(1))+l6*cos(joint_position.at(1)+joint_position.at(2))+ l7)*sin(joint_position.at(0));
  double z6 = l2 + l3 + sqrt(l4*l4+l5*l5)*sin(alpha-joint_position.at(1))-l6*sin(joint_position.at(1)+joint_position.at(2));

  goal_position_.clear();
  goal_position_.push_back(x6);
  goal_position_.push_back(y6);
  goal_position_.push_back(z6);
  return true;
}


bool OmtbKinematics::inverse_kinematics(std::vector<double> goal_position)
{
  double joint1 = 0;
  double joint2 = 0;
  double joint3 = 0;
  double joint4 = 0;
  double r;
  if (abs(goal_position.at(0)-l1)<error && abs(goal_position.at(1))<error) {joint1 = 0;r = 0;}
  else if (abs(goal_position.at(0)-l1)<error && goal_position.at(1)>0) {joint1 = PI/2; r = goal_position.at(1)-l7;}
  else if (abs(goal_position.at(0)-l1)<error && goal_position.at(1)<0) {joint1 = -PI/2; r = -goal_position.at(1)-l7;}
  else {joint1 = atan2(goal_position.at(1),(goal_position.at(0)-l1));r = (goal_position.at(0)-l1)/cos(joint1)-l7;}
  double left = r*r+(goal_position.at(2)-l2-l3)*(goal_position.at(2)-l2-l3)-l4*l4-l5*l5-l6*l6;
  double right = 2*l6*sqrt(l4*l4+l5*l5);
  double cos_alphajoint3 = 1;
  double sin_alphajoint3 = 0;
  joint_position_.clear();
  if (abs(left)>=abs(right))
  {
    cout<<"Out of range!!!!"<<endl;
    return false;
  }
  else
  {
    cos_alphajoint3 = left/right;
    sin_alphajoint3 = sqrt(1-cos_alphajoint3*cos_alphajoint3);
    joint3 = atan2(sin_alphajoint3, cos_alphajoint3) - alpha;
    double beta = atan2(l6*sin_alphajoint3, sqrt(l4*l4+l5*l5)+l6*cos_alphajoint3);
    joint2 = alpha - beta - atan2(goal_position.at(2)-l2-l3, r);
    joint4 = 0 - joint2 - joint3;
    joint_position_.push_back(joint1);
    joint_position_.push_back(joint2);
    joint_position_.push_back(joint3);
    joint_position_.push_back(joint4);
    return true;
  }
}
