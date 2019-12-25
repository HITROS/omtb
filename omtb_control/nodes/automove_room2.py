#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi
import PyKDL
from math import pi
from math import sin
from math import atan2

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


def right_linear(linear):

    if linear > 0 and linear < 0.005:
        linear = 0.005
    elif linear > 0.2:
        linear = 0.2
    elif linear < 0 and linear > -0.005:
        linear = -0.005
    elif linear < -0.2:
        linear = -0.2
    return linear


def right_angular(angular):

    if angular > 0 and angular < 0.005:
        angular = 0.005
    elif angular > 0.2:
        angular = 0.2
    elif angular < 0 and angular > -0.005:
        angular = -0.005
    elif angular < -0.2:
        angular = -0.2
    return angular


def not_done(x, y, goal_x, goal_y):
    if abs(goal_x - x)<0.01 and abs(goal_y - y)<0.01:
        return False
    else:
        return True


class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)
        tf_prefix = rospy.get_param('tf_prefix', 'robot1')
        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # How fast will we update the robot's movement?
        rate = 200
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.2 meters per second 
        linear_speed = 0.2
        
        # Set the travel points in meters
        if tf_prefix == 'robot1':
            goal_x = [ 2.0,  2.0, -2.0, -2.0,  0.0]
            goal_y = [ 0.0, -2.0, -2.0,  0.0,  0.0]
        elif tf_prefix == 'robot2':
            goal_x = [ 2.0,  2.0, -2.0, -2.0,  0.0]
            goal_y = [ 2.0,  0.0,  0.0,  2.0,  2.0]
        else:
            goal_x = [ 2.0,  2.0, -2.0, -2.0,  2.0,  2.0,  0.0]
            goal_y = [ 0.0, -2.0, -2.0,  2.0,  2.0,  0.0,  0.0]
        # Set the rotation speed in radians per second
        angular_speed = 0.01
        
        # Set the angular tolerance in degrees converted to radians
        angular_tolerance = radians(5)
        distance_tolerance = 0.01
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/'+tf_prefix+'/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/'+tf_prefix+'/base_footprint', rospy.Time(), rospy.Duration(1))
            self.base_frame = '/'+tf_prefix+'/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/'+tf_prefix+'/base_link', rospy.Time(), rospy.Duration(1))
                self.base_frame = '/'+tf_prefix+'/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        
        # Initialize the position variable as a Point type
        position = Point()
        for i in range(len(goal_x)):
            print("goal: x=%f, y=%f"  %(goal_x[i],goal_y[i]))
            move_cmd = Twist()
            (position, rotation) = self.get_odom()
            angle_to_goal = 0
            while not_done(position.x, position.y, goal_x[i], goal_y[i]):
                inc_x = goal_x[i] - position.x
                inc_y = goal_y[i] - position.y
                last_goal = angle_to_goal
                angle_to_goal = atan2(inc_y, inc_x)
                alpha = angle_to_goal - last_goal
                if abs(alpha) > 1.5*pi:
                    if alpha > 0:
                        angle_to_goal -= 2 * pi
                    else:
                        angle_to_goal += 2 * pi
#                print("goal:")
#                print(angle_to_goal)
#                print(rotation)
                if abs(angle_to_goal - rotation) > abs(angular_tolerance) or abs(angle_to_goal - rotation) > pi - abs(angular_tolerance):
                    move_cmd.linear.x = 0
                    if angle_to_goal > rotation:
                        move_cmd.angular.z = 0.3
                    else:
                        move_cmd.angular.z = -0.3
                else:
                    move_cmd.linear.x = 0.1
                    move_cmd.angular.z = 0.0
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                last_rotation = rotation
                (position, rotation) = self.get_odom()
                delta = rotation - last_rotation
                if abs(delta) > pi:
                    if delta > 0:
                        rotation -= 2 * pi
                    else:
                        rotation += 2 * pi
                # rotation = normalize_angle(rotation)

            # Stop the robot before the next leg
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(2)
        print("All done!")
        # Stop the robot for good
        self.cmd_vel.publish(Twist())
        
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans), quat_to_angle(Quaternion(*rot))
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

