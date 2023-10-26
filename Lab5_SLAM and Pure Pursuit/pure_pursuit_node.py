#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO CHECK: include needed ROS msg type headers and libraries
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
import math

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers

        self.LOOKAHEAD = 0.8
        self.velocity = 1.0
        self.constant = 0.8
        self.waypoints = np.array(self.read_ground_truth_points())

        drive_topic = '/drive'
        odometry_topic = '/ego_racecar/odom'

        # Odometry Subscriber
        self.odometry_subscription = self.create_subscription(
            Odometry, 
            odometry_topic,
            self.pose_callback, 
            10)
        self.odometry_subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        #init buffer and listener for make pose transforms on waypoint
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)


    def read_ground_truth_points(self):
        path = "src/pure_pursuit/resource/"
        fname = "reference_waypoints"

        waypoints = []
        with open(path + fname + '.csv', 'r') as file:
            lines = file.readlines()
            for line in lines:
                data = line.strip().split(',')
                waypoints.append([float(val) for val in data])
        return waypoints


    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture

        x_car = pose_msg.pose.pose.position.x
        y_car = pose_msg.pose.pose.position.y

        # Compute the squared differences
        deltaX = self.waypoints[:,0] - x_car
        deltaY = self.waypoints[:,1] - y_car
        distances = np.sqrt(deltaX**2 + deltaY**2)
        min_idx = np.argmin(distances)

        self.waypoints = self.waypoints[min_idx:,:]
        distances = distances[min_idx:]

        # Boolean array for points on and outside of the lookahead
        outside_L = distances >= self.LOOKAHEAD

        # Find point with smallest distance from car
        filtered_distances = distances[outside_L] #filter distances
        min_index_outside_L = np.argmin(filtered_distances) #index of smalled distance in filtered
        filtered_index_array = np.arange(distances.size)[outside_L] #create array of indicies {0, 1, 2, 3..} and filter based on outside_L
        goal_waypoint_index = filtered_index_array[min_index_outside_L]
        
        # with index obtain the goal waypoint
        goal_waypoint = (self.waypoints[:,0][goal_waypoint_index], self.waypoints[:,1][goal_waypoint_index])
        self.publish_point_marker(goal_waypoint[0], goal_waypoint[1])

        # TODO: transform goal point to vehicle frame of reference
        transformed_point = self.transform_point(goal_waypoint, pose_msg)
        
        # goal_waypoint_tfCar = self.waypointTransform(goal_waypoint, pose_msg)
        if transformed_point:
            x_waypoint_tf = transformed_point.position.x
            y_waypoint_tf = transformed_point.position.y

            # TODO: calculate curvature/steering angle
            # get steering angle based on arc formula
            angle = self.steerToWaypoint(0, 0, x_waypoint_tf, y_waypoint_tf)
            
            # limiting angle. change this value based on sim performance
            max_angle = 60
            if angle >= np.radians(max_angle):
                angle = np.radians(max_angle)
            print("angle:", angle)

            # TODO: publish drive message, don't forget to limit the steering angle.
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.drive.speed = self.velocity
            drive_msg.drive.steering_angle = angle
            self.publisher_.publish(drive_msg)


    def transform_point(self, goal_point, pose_msg):
        #convert our goal_waypoint into a PoseStamped messaeg
        point_stamped = Pose()
        point_stamped.position.x  = goal_point[0]
        point_stamped.position.y = goal_point[1]
        point_stamped.position.z = 0.0
        point_stamped.orientation.x = 0.0
        point_stamped.orientation.y = 0.0
        point_stamped.orientation.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform('ego_racecar/base_link',
                                                    'map',
                                                    rclpy.time.Time()
                                                    )
            goal_waypoint_tfCar = tf2_geometry_msgs.do_transform_pose(point_stamped, transform)
            return goal_waypoint_tfCar
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform ego_racecar/base_link to map: {ex}')
            return

    def steerToWaypoint(self, x_car, y_car, x_goal, y_goal):
        X = x_goal - x_car
        Y = y_goal - y_car
        dist_euc = math.sqrt(X**2+Y**2)
        steering_angle = (2*Y)/(dist_euc**2) * self.constant
        return steering_angle

    def publish_point_marker(self, x, y, frame_id='map'):

        # Create a Marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id  # Set the frame ID (default: base_link)
        marker_msg.header.stamp = self.get_clock().now().to_msg()
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
        marker_msg.color.g = 255.0  
        marker_msg.color.b = 0.0

        # Publish the Marker message
        self.marker_publisher.publish(marker_msg)


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()