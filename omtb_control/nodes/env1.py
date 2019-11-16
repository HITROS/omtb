#!/usr/bin/env python

import rospy
from math import pow, atan2, sqrt
from tf.transformations import *

import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState

import threading

# Navigation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Manipulator
from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose

# AR Markers
from ar_track_alvar_msgs.msg import AlvarMarker
from ar_track_alvar_msgs.msg import AlvarMarkers


def main():
    rospy.init_node('pick_and_place_state_machine')
    namespace = rospy.get_param("~robot_name")
    planning_group = rospy.get_param("~planning_group")
#    namespace = "robot1"
#    planning_group = "arm"

    # Create the sub SMACH state machine
    task_center = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    # Open the container
    with task_center:
        task_center.userdata.planning_group = planning_group
        def joint_position_request_cb(userdata, request):
            joint = JointPosition()
            joint.position = userdata.input_position
            joint.max_velocity_scaling_factor = 1
            joint.max_accelerations_scaling_factor = 1

            request.planning_group = userdata.input_planning_group
            request.joint_position = joint
            return request

        def joint_position_response_cb(userdata, response):
            if response.is_planned == False:
                return 'aborted'
            else:
                rospy.sleep(3.)
                return 'succeeded'

        def eef_pose_request_cb(userdata, request):
            eef = KinematicsPose()
            eef.pose = userdata.input_pose
            rospy.loginfo('eef.position.x : %f', eef.pose.position.x)
            rospy.loginfo('eef.position.y : %f', eef.pose.position.y)
            rospy.loginfo('eef.position.z : %f', eef.pose.position.z)
            eef.max_velocity_scaling_factor = 0.01
            eef.max_accelerations_scaling_factor = 0.01
            eef.tolerance = userdata.input_tolerance

            request.planning_group = userdata.input_planning_group
            request.kinematics_pose = eef
            return request

        def align_arm_with_object_response_cb(userdata, response):
            if response.is_planned == False:
                task_center.userdata.align_arm_with_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', task_center.userdata.align_arm_with_object_tolerance)
                return 'aborted'
            else:
                OFFSET_FOR_STRETCH = 0.000
                task_center.userdata.object_pose1.position.x += OFFSET_FOR_STRETCH
                rospy.sleep(3.)
                return 'succeeded'

        def close_to_object_response_cb(userdata, response):
            if response.is_planned == False:
                task_center.userdata.close_to_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', task_center.userdata.close_to_object_tolerance)
                return 'aborted'
            else:
                OFFSET_FOR_OBJECT_HEIGHT = 0.000
                task_center.userdata.object_pose.position.z += OFFSET_FOR_OBJECT_HEIGHT
                rospy.sleep(3.)
                return 'succeeded'

        def pick_up_object_response_cb(userdata, response):
            if response.is_planned == False:
                task_center.userdata.pick_up_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', task_center.userdata.pick_up_object_tolerance)
                return 'aborted'
            else:
                rospy.sleep(3.)
                return 'succeeded'

        def gripper_request_cb(userdata, request):
            joint = JointPosition()
            joint.position = userdata.input_gripper
            joint.max_velocity_scaling_factor = 0.1
            joint.max_accelerations_scaling_factor = 0.1

            request.planning_group = userdata.input_planning_group
            request.joint_position = joint
            return request

        def gripper_response_cb(userdata, response):
            rospy.sleep(1.)
            return 'succeeded'

        def get_pose(position):
            object_pose = Pose()
            object_pose.position.x = position[0] - 0.012
            object_pose.position.y = position[1]
            object_pose.position.z = position[2]
            dist = math.sqrt((object_pose.position.x * object_pose.position.x) +
                             (object_pose.position.y * object_pose.position.y))

            if object_pose.position.y > 0:
                yaw = math.acos(object_pose.position.x / dist)
            else:
                yaw = (-1) * math.acos(object_pose.position.x / dist)

            roll = 0.0
            pitch = 0.0
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)

            object_pose.orientation.w = cy * cr * cp + sy * sr * sp
            object_pose.orientation.x = cy * sr * cp - sy * cr * sp
            object_pose.orientation.y = cy * cr * sp + sy * sr * cp
            object_pose.orientation.z = sy * cr * cp - cy * sr * sp
            return object_pose



        # Add states to the container
        task_center.userdata.init_position = [0, 0.5, 0.1, -0.1]
        task_center.userdata.init_tolerance = 0.0001
        smach.StateMachine.add('SET_INIT_POSITION',
                                ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                request_cb=joint_position_request_cb,
                                                response_cb=joint_position_response_cb,
                                                input_keys=['input_planning_group',
                                                            'input_position',
                                                            'input_tolerance']),
                                transitions={'succeeded':'succeeded',
                                             'aborted':'aborted'},
                                remapping={'input_planning_group':'planning_group',
                                            'input_position':'init_position',
                                            'input_tolerance':'init_tolerance'})



#        task_center.userdata.open_gripper = [0.15]
#        smach.StateMachine.add('SET_INIT_POSITION',
#                                ServiceState('gripper',
#                                                SetJointPosition,
#                                                request_cb=gripper_request_cb,
#                                                response_cb=gripper_response_cb,
#                                                input_keys=['input_planning_group',
#                                                            'input_gripper']),
#                                transitions={'succeeded':'ALIGN_ARM_WITH_OBJECT'},
#                                remapping={'input_planning_group':'planning_group',
#                                        'input_gripper':'open_gripper'})

#        position1 = [0.235, 0, 0.165]
#        task_center.userdata.align_arm_with_object_tolerance = 0.002
#        task_center.userdata.object_pose1 = get_pose(position1)
#        smach.StateMachine.add('ALIGN_ARM_WITH_OBJECT',
#                                ServiceState(planning_group + '/moveit/set_kinematics_pose',
#                                                SetKinematicsPose,
#                                                request_cb=eef_pose_request_cb,
#                                                response_cb=align_arm_with_object_response_cb,
#                                                input_keys=['input_planning_group',
#                                                            'input_pose',
#                                                            'input_tolerance']),
#                               transitions={'succeeded':'GRIP_OBJECT',
#                                            'aborted':'aborted'},
#                               remapping={'input_planning_group':'planning_group',
#                                        'input_pose':'object_pose1',
#                                        'input_tolerance':'align_arm_with_object_tolerance'})

#        task_center.userdata.close_gripper = [-0.1]
#        smach.StateMachine.add('GRIP_OBJECT',
#                                ServiceState('gripper',
#                                                SetJointPosition,
#                                                request_cb=gripper_request_cb,
#                                                response_cb=gripper_response_cb,
#                                                input_keys=['input_planning_group',
#                                                            'input_gripper']),
#                                transitions={'succeeded':'PICK_UP_OBJECT1'},
#                                remapping={'input_planning_group':'planning_group',
#                                        'input_gripper':'close_gripper'})

#        position2 = [0.235, 0, 0.20]
#        task_center.userdata.align_arm_with_object_tolerance = 0.002
#        task_center.userdata.object_pose2 = get_pose(position2)
#        smach.StateMachine.add('PICK_UP_OBJECT1',
#                                ServiceState(planning_group + '/moveit/set_kinematics_pose',
#                                                SetKinematicsPose,
#                                                request_cb=eef_pose_request_cb,
#                                                response_cb=align_arm_with_object_response_cb,
#                                                input_keys=['input_planning_group',
#                                                            'input_pose',
#                                                            'input_tolerance']),
#                               transitions={'succeeded':'PICK_UP_OBJECT2',
#                                            'aborted':'aborted'},
#                               remapping={'input_planning_group':'planning_group',
#                                        'input_pose':'object_pose2',
#                                        'input_tolerance':'align_arm_with_object_tolerance'})


#        position3 = [0.235 , 0.05, 0.20]
#        task_center.userdata.align_arm_with_object_tolerance = 0.01
#        task_center.userdata.object_pose3 = get_pose(position3)
#        smach.StateMachine.add('PICK_UP_OBJECT2',
#                                ServiceState(planning_group + '/moveit/set_kinematics_pose',
#                                                SetKinematicsPose,
#                                                request_cb=eef_pose_request_cb,
#                                                response_cb=align_arm_with_object_response_cb,
#                                                input_keys=['input_planning_group',
#                                                            'input_pose',
#                                                            'input_tolerance']),
#                               transitions={'succeeded':'succeeded',
#                                            'aborted':'aborted'},
#                               remapping={'input_planning_group':'planning_group',
#                                        'input_pose':'object_pose3',
#                                        'input_tolerance':'align_arm_with_object_tolerance'})




    sis = smach_ros.IntrospectionServer('server_name', task_center, '/TASKS_CENTER')
    sis.start()

    # Execute SMACH plan
    outcome = task_center.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
