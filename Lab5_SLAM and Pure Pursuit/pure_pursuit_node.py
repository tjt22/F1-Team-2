#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# TODO CHECK: include needed ROS msg type headers and libraries
# make sure these are in your package and cmake files
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.LOOKAHEAD = 1.5 # meters
        self.velocity = 0.5 # m/s
        self.subscription = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)
        self.Ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        #init buffer and listener for make pose transforms on waypoint
        self.tf_buffer = tf2_ros.Buffer(rclpy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #self.subscription = self.node.create_subscription(MarkerArray, 'visualization_marker_array',self.viz_callback,10)
        FILEPATH = "ENTER_FILEPATH_HERE"
        self.waypoints = np.loadtxt(FILEPATH,delimiter=",",dtype={'names': ('x','y','z')},usecols=(0,1,2))

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
        
        # with index obtain the goal waypoint
        goal_waypoint = (self.waypoints['x'][goal_waypoint_index], self.waypoints['y'][goal_waypoint_index])
        goal_waypoint_tfCar = self.waypointTransform(goal_waypoint, msg)
        x_waypoint_tf = goal_waypoint_tfCar.pose.position.x
        y_waypoint_tf = goal_waypoint_tfCar.pose.position.y
        
        # get steering angle based on arc formula
        angle = self.steerToWaypoint(x_car, y_car, x_waypoint_tf, y_waypoint_tf)

        # limiting angle. change this value based on sim performance
        max_angle = 60
        if angle >= np.radians(max_angle):
            angle = np.radians(max_angle)
            
        print(angle)
        # show's goal point on Rviz
        publish_point_marker(x_waypoint_tf,y_waypoint_tf)
        
        #shut up and drive https://www.youtube.com/watch?v=up7pvPqNkuU
        #drive and steer messages
        angle_msg = AckermannDriveStamped()
        angle_msg.header.stamp = self.get_clock().now().to_msg()
        angle_msg.drive.steering_angle = angle
        self.Ackermann_publisher.publish(angle_msg)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.velocity
        self.Ackermann_publisher.publish(drive_msg)
        
         
    def waypointTransform(self, goal_waypoint, msg):
        #convert our goal_waypoint into a PoseStamped messaeg
        goalWaypointPose = Pose()
        goalWaypointPose.position.x  = goal_waypoint[0]
        goalWaypointPose.position.y = goal_waypoint[1]
        goalWaypointPose.position.z = 0
        goalWaypointPose.orientation.x = 0
        goalWaypointPose.orientation.y = 0
        goalWaypointPose.orientation.z = 0

        goal_waypoint_mapFrame = PoseStamped()
        goal_waypoint_mapFrame.header = msg.pose.header
        goal_waypoint_mapFrame.pose   = goalWaypointPose

        #do the transform
        transform = self.tf_buffer.lookup_transform('ego_racecar/base_link',
                                        'map',
                                        msg.pose.header.stamp,
                                        rclpy.Duration(1.0)
                                        )
        goal_waypoint_tfCar = tf2_geometry_msgs.do_transform_pose(goal_waypoint_mapFrame, transform)
        return goal_waypoint_tfCar


    def steerToWaypoint(self, x_car, y_car, x_goal, y_goal):
        X = x_goal - x_car
        Y = y_goal - y_car
        dist_euc = math.sqrt(X**2+Y**2)
        steering_angle = (2*abs(Y))/(dist_euc**2)
        return steering_angle
        
    def publish_point_marker(x, y, frame_id='map'):

        # Create a Marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id  # Set the frame ID (default: base_link)
        marker_msg.header.stamp = node.get_clock().now().to_msg()
        marker_msg.ns = 'points'
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x  # X-coordinate of the point
        marker_msg.pose.position.y = y  # Y-coordinate of the point
        marker_msg.pose.position.z = 0.0  # Z-coordinate of the point
        marker_msg.scale.x = 0.2  # Scale of the marker (adjust as needed)
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.a = 1.0  # Alpha (transparency)
        marker_msg.color.r = 0.0  
        marker_msg.color.g = 255.0  # Green marker
        marker_msg.color.b = 0.0

        # Publish the Marker message
        publisher.publish(marker_msg)
        node.get_logger().info('Marker published')


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
