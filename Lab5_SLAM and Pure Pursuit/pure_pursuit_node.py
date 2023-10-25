#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# TODO CHECK: include needed ROS msg type headers and libraries
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import Odometry


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers

        self.LOOKAHEAD = 1.5 # meters
        self.velocity = 1.5 # m/s
        self.msg.velocity = 1.5
        tf_buffer = tf2_rclpy.Buffer(rclpy.Duration(100.0))
        tf_listener = tf2_rclpy.TransformListener(self.tf_buffer)
        self.subscription = self.node.create_subscription(
            Odometry,
            'ego_racecar/odom',  # Replace 'odom' with the actual topic name
            self.odom_callback,
            10
        )
        self.subscription = self.node.create_subscription(MarkerArray, 'visualization_marker_array',self.viz_callback,10)
        self.waypoints = np.loadtxt('filepath',delimiter=",",dtype={'names': ('x','y','z')},usecols=(0,1,2))

    def odom_callback(self, msg):
        x_car = msg.pose.pose.position.x
        y_car = msg.pose.pose.position.y
        qx_car = msg.pose.pose.orientation.x
        qy_car = msg.pose.pose.orientation.y
        qz_car = msg.pose.pose.orientation.z
        qw_car = msg.pose.pose.orientation.w

        # Compute the squared differences
        deltaX = self.waypoints['x'] - x_car
        deltaY = self.waypoints['y'] - y_car
        distances = np.sqrt(deltaX**2 + deltaY**2)

        # Boolean array for points on and outside of the lookahead
        outside_L = distances >= self.LOOKAHEAD

        # Find point with smallest distance from car
        filtered_distances = distances[outside_L] #filter distances
        min_index_outside_L = np.argmin(filtered_distances) #index of smalled distance in filtered
        filtered_index_array = np.arange(distances.size)[outside_L] #create array of indicies {0, 1, 2, 3..} and filter based on outside_L
        goal_waypoint_index = filtered_index_array[min_index_outside_L]
        
        #with index obtain the goal waypoint
        goal_waypoint = (self.waypoints['x'][goal_waypoint_index], self.waypoints['y'][goal_waypoint_index])
        #TODO: call our transformation function
            # right now it's pose_callback, but maybe it should be a function like
            # transformGoal(goal_waypoint, msg):
            #   turn the  goal_waypoint into a PoseStamped message
            #   do the transform (with the buffer, lookup, etc)
 



    def viz_callback(self, msg):
        waypoints = msg.marker_array

    def pose_callback(self, msg):
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # loop through points in
        deltaX = self.waypoints['x'] - x_car
        deltaY = self.waypoints['Y'] - y_car


        # TODO: transform goal point to vehicle frame of reference
        quaternion_car = (qx_car,qy_car,qz_car,qw_car)
        transform = tf_buffer.lookup_transform('ego_racecar/base_link',
                                               'map',
                                               msg.pose.header.stamp,
                                               rclpy.Duration(1.0)
                                               )
        pose_transformed = tf2_geometry_msgs.do_transform_pose(#, transform)

        # TODO: calculate curvature/steering angle
        X = x_goal - x_car
        Y = y_goal - y_car
        dist_euc = math.sqrt(X**2+Y**2)
        L = 200
        steering_angle = (2*abs(Y))/(L**2)

        # TODO: publish drive message, don't forget to limit the steering angle.


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''