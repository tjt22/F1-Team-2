"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

# TODO: import as you need
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# class def for tree nodes
# It's up to you if you want to use this
class MyNode(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = 0 # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('RRT_Node')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        # pose_topic = "/ego_racecar/odom"
        pose_topic = "/pf/pose/odom"
        scan_topic = "/scan"
        drive_topic = "/drive"
        grid_topic = "/nav_msgs/OccupancyGrid"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            Odometry, # PoseStamped,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # # publishers
        # # TODO: create a drive message publisher, and other publishers that you might need
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.grid_publisher_ = self.create_publisher(OccupancyGrid, grid_topic, 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        # class attributes
        # TODO: maybe create your occupancy grid here
        # self.grid_size_half_x = 50
        # self.grid_size_half_y = 50
        self.grid_size_half_x = 20
        self.grid_size_half_y = 20
        self.grid_size_x = self.grid_size_half_x*2
        self.grid_size_y = self.grid_size_half_y*2
        self.resolution = 0.1
        self.occupancy_grid = np.zeros((self.grid_size_y,self.grid_size_x), dtype=np.int32)
        self.waypoints = np.array(self.read_ground_truth_points())

        self.pose_x = None
        self.pose_y = None
        self.orientation = None
        self.step_size = 1.0
        self.goal_tolerance = 1.5
        # self.goal_x = 5.0
        self.goal_x = 3.0
        self.goal_y = 0.0
        self.rrt_iter_max = 100
        self.neighbor_tolerance = 1.0

        self.marker = 0
        
        self.max_steering_angle = np.pi/4.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        # self.br = TransformBroadcaster(self)
        # self.publish_transforms()


    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        self.occupancy_grid = np.zeros((self.grid_size_y,self.grid_size_x), dtype=np.int32)
            
        ranges = scan_msg.ranges
        bubble = 0.10

        for i, distance in enumerate(ranges):
            if distance == float('inf') or distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            distance -= bubble

            obstacle_x = distance * math.cos(angle)
            obstacle_y = distance * math.sin(angle)

            grid_x = int(obstacle_x/self.resolution) + self.grid_size_half_x
            grid_y = int(obstacle_y/self.resolution) + self.grid_size_half_y

            if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
                self.occupancy_grid[grid_y][grid_x] = 100 

        
        publish_grid = OccupancyGrid()
        publish_grid.info.resolution = self.resolution
        publish_grid.info.width = self.grid_size_x 
        publish_grid.info.height = self.grid_size_y  
        publish_grid.data = (self.occupancy_grid.flatten()).tolist()

        publish_grid.header.frame_id = "laser"
        publish_grid.header.stamp = self.get_clock().now().to_msg()

        publish_grid.info.resolution = self.resolution

        publish_grid.info.origin.position.x = - float(self.grid_size_half_x) * self.resolution
        publish_grid.info.origin.position.y = - float(self.grid_size_half_y) * self.resolution
        publish_grid.info.origin.position.z = 0.0

        self.grid_publisher_.publish(publish_grid)
        # print("grid sent")

    

    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        self.pose_x = pose_msg.pose.pose.position.x
        self.pose_y = pose_msg.pose.pose.position.y
        self.orientation = pose_msg.pose.pose.orientation
        print("current position:", self.pose_x, self.pose_y)

        deltaX = self.waypoints[:,0] - self.pose_x
        deltaY = self.waypoints[:,1] - self.pose_y
        distances = np.sqrt(deltaX**2 + deltaY**2)
        min_idx = (np.argmin(distances) + 3) % len(self.waypoints)
        self.current_index = min_idx

        goal_x_global = self.waypoints[self.current_index, 0]
        goal_y_global = self.waypoints[self.current_index, 1]

        self.marker = 0

        # self.publish_point_marker(goal_x_global, goal_y_global, frame_id="map", corlor_r=255.0, corlor_g=0.0, color_b=0.0)


        # o = pose_msg.pose.pose.orientation # Quaternion type, x y z and w

        # current_yaw = np.arctan2(
        #    2.0 * (o.w * o.z + 
        #           o.x * o.y),
        #    1.0 - 2.0 * (o.y ** 2 + o.z ** 2)
        # )
        
        # dx = goal_x_global - pose_msg.pose.pose.position.x
        # dy = goal_y_global - pose_msg.pose.pose.position.y
        # goal_x = dx * np.cos(-current_yaw) - dy * np.sin(-current_yaw)
        # goal_y = dx * np.sin(-current_yaw) + dy * np.cos(-current_yaw)

        goal_stamped = Pose()
        goal_stamped.position.x = goal_x_global
        goal_stamped.position.y = goal_y_global
        goal_stamped.position.z = 0.0
        goal_stamped.orientation.x = 0.0
        goal_stamped.orientation.y = 0.0
        goal_stamped.orientation.z = 0.0
        car_goal = self.transform_point(goal_stamped, from_frame='map', to_frame='laser')

        
        if car_goal:
        # if True:
            self.publish_point_marker(car_goal.position.x, car_goal.position.y, frame_id="laser", corlor_r=0.0, corlor_g=255.0, color_b=0.0)
            self.goal_x = car_goal.position.x
            self.goal_y = car_goal.position.y
        
        
        # if self.pose_x == None or not self.pose_y == None:
        #     return

        root = MyNode()
        root.x = 0.0
        root.y = 0.0
        root.is_root = True
        tree = [root]

        # self.publish_point_marker(self.goal_x, self.goal_y, frame_id="ego_racecar/base_link", corlor_r=255.0, corlor_g=0.0, color_b=0.0)
        print("goal:", self.goal_x, self.goal_y)
        
        for iter in range(self.rrt_iter_max): 
            sampled_point = self.sample()
            # self.publish_point_marker(sampled_point[0], sampled_point[1], frame_id="ego_racecar/base_link", corlor_r=255.0, corlor_g=0.0, color_b=0.0)
        
            nearest_node_idx = self.nearest(tree, sampled_point)
            nearest_node = tree[nearest_node_idx]
            # self.publish_point_marker(nearest_node.x, nearest_node.y, frame_id="ego_racecar/base_link", corlor_r=255.0, corlor_g=0.0, color_b=0.0)

            new_node = self.steer(nearest_node, sampled_point)
            if not new_node:
                continue
            # self.publish_point_marker(new_node.x, new_node.y, frame_id="ego_racecar/base_link", corlor_r=0.0, corlor_g=255.0, color_b=0.0)

            if self.check_collision(nearest_node, new_node):
                neighbors = self.near(tree, new_node)
                self.choose_parent(tree, neighbors, new_node)
                tree.append(new_node)
                # new_node.parent = nearest_node
                self.rewire(tree, neighbors, new_node)

                

                if self.is_goal(new_node, self.goal_x, self.goal_y):
                    print("Goal reached!")
                    path = self.find_path(tree, new_node)

                    target_node = path[1]
                    dx = target_node.x
                    dy = target_node.y
                    steering_angle = math.atan2(dy, dx) * 0.35

                    if abs(steering_angle) > self.max_steering_angle:
                        if steering_angle > 0:
                            steering_angle = self.max_steering_angle
                        else:
                            steering_angle = -self.max_steering_angle
                    
                    if steering_angle > self.max_steering_angle / 1.1:
                        speed = 1.0
                    else:
                        speed = 2.5

                    drive_msg = AckermannDriveStamped()
                    drive_msg.header.stamp = self.get_clock().now().to_msg()
                    drive_msg.header.frame_id = "laser"
                    drive_msg.drive.steering_angle = steering_angle
                    drive_msg.drive.speed = speed

                    self.publisher_.publish(drive_msg)

                    
                    for idx in range(len(path)):
                        target_node = path[idx]

                        self.publish_point_marker(target_node.x, target_node.y, frame_id="laser", corlor_r=0.0, corlor_g=0.0, color_b=255.0)

                    #     # dx = target_node.x
                    #     # dy = target_node.y
                    #     # steering_angle = math.atan2(dy, dx)

                    #     # drive_msg = AckermannDriveStamped()
                    #     # drive_msg.header.stamp = self.get_clock().now().to_msg()
                    #     # drive_msg.header.frame_id = "ego_racecar/base_link"
                    #     # drive_msg.drive.steering_angle = steering_angle
                    #     # drive_msg.drive.speed = 1.0 

                    #     # self.publisher_.publish(drive_msg)
                    

                    self.marker = 0
                    break
            else:
                # print("Collision")
                pass

        # return None

    def read_ground_truth_points(self):
        path = "src/lab-6-motion-planning-autonomous-anonymous/lab6_pkg/resource/"
        fname = "pacday2"

        waypoints = []
        with open(path + fname + '.csv', 'r') as file:
            lines = file.readlines()
            for line in lines:
                data = line.strip().split(',')
                waypoints.append([float(val) for val in data])
        return waypoints

    def transform_point(self, point_stamped, from_frame='laser', to_frame='map'):
        try:
            transform = self.tf_buffer.lookup_transform(
                                                    to_frame,
                                                    from_frame,
                                                    rclpy.time.Time()
                                                    )
            goal_waypoint_tfCar = tf2_geometry_msgs.do_transform_pose(point_stamped, transform)
            return goal_waypoint_tfCar
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform ego_racecar/base_link to map: {ex}')
            return

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        # x = np.random.random_integers(low=0.0,high=self.grid_size_x-1)
        x = np.random.random_integers(low=self.grid_size_half_x, high=self.grid_size_x-1)
        y = np.random.random_integers(low=0.0,high=self.grid_size_y-1)
        # y = np.random.random_integers(low=self.grid_size_half_y,high=self.grid_size_y-1)

        while self.occupancy_grid[y,x]:
            # x = np.random.random_integers(low=0.0,high=self.grid_size_x-1)
            x = np.random.random_integers(low=self.grid_size_half_x, high=self.grid_size_x-1)
            y = np.random.random_integers(low=0.0,high=self.grid_size_y-1)
            # y = np.random.random_integers(low=self.grid_size_half_y,high=self.grid_size_y-1)

        pose_x, pose_y = self.grid_to_pose(x, y)

        return (pose_x, pose_y)

    def grid_to_pose(self, grid_x, grid_y):
        pose_x = (grid_x - self.grid_size_half_x) * self.resolution
        pose_y = (grid_y - self.grid_size_half_y) * self.resolution
        return pose_x, pose_y

    def pose_to_grid(self, pose_x, pose_y):
        grid_x = int(pose_x/self.resolution) + self.grid_size_half_x
        grid_y = int(pose_y/self.resolution) + self.grid_size_half_y
        return grid_x, grid_y

    def publish_point_marker(self, x, y, frame_id='map', corlor_r=255.0, corlor_g=0.0, color_b=0.0):

        # Create a Marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id  # Set the frame ID (default: base_link)
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = 'points'
        # marker_msg.id = 0
        marker_msg.id = self.marker
        self.marker += 1
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x  # X-coordinate of the point
        marker_msg.pose.position.y = y  # Y-coordinate of the point
        marker_msg.pose.position.z = 0.0  # Z-coordinate of the point
        marker_msg.scale.x = 0.2  # Scale of the marker (adjust as needed)
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2
        marker_msg.color.a = 1.0  # Alpha (transparency)
        marker_msg.color.r = corlor_r
        marker_msg.color.g = corlor_g 
        marker_msg.color.b = color_b

        # Publish the Marker message
        self.marker_publisher.publish(marker_msg)
    
    def clear_marker(self, frame_id='map', id=0):
        clear_marker_msg = Marker()
        clear_marker_msg.header.frame_id = frame_id 
        clear_marker_msg.id = id 
        clear_marker_msg.action = Marker.DELETE 
        self.publisher.publish(clear_marker_msg)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        min_distance = float('inf')

        for i in range(len(tree)):
            node = tree[i]
            distance = np.linalg.norm(np.array([node.x - sampled_point[0], node.y - sampled_point[1]]))
            if distance < min_distance:
                min_distance = distance
                nearest_node = i

        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        new_node = None

        direction = np.array([sampled_point[0] - nearest_node.x, sampled_point[1] - nearest_node.y])
        
        if not np.linalg.norm(direction):
            return None
        
        unit_direction = direction / np.linalg.norm(direction)

        new_node = MyNode()
        new_node.x = nearest_node.x + self.step_size * unit_direction[0]
        new_node.y = nearest_node.y + self.step_size * unit_direction[1]
        new_node.parent = nearest_node

        grid_x, grid_y = self.pose_to_grid(new_node.x, new_node.y)
        
        if grid_x > self.grid_size_x or grid_y > self.grid_size_y:
            return None
        
        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """

        num_points = 100
        x_path = np.linspace(nearest_node.x, new_node.x, num_points)
        y_path = np.linspace(nearest_node.y, new_node.y, num_points)

        for x, y in list(zip(x_path, y_path)):
            grid_x, grid_y = self.pose_to_grid(x,y)
            # self.publish_point_marker(x, y, frame_id="ego_racecar/base_link", corlor_r=0.0, corlor_g=0.0, color_b=255.0)

            if grid_x >= self.grid_size_x or grid_y >= self.grid_size_y:
                return False

            if self.occupancy_grid[grid_y][grid_x]:
                return False

        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        
        distance = np.linalg.norm([latest_added_node.x - goal_x, latest_added_node.y - goal_y])

        if distance <= self.goal_tolerance:
            return True
    
        return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []

        cur_ptr = latest_added_node
        while cur_ptr:
            path.append(cur_ptr)
            cur_ptr = cur_ptr.parent
    
        return path[::-1]



    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        cost = 0
        while node.parent is not None:
            cost += self.line_cost(node, node.parent)
            node = node.parent
        return cost


    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """

        return np.linalg.norm(np.array([n1.x - n2.x, n1.y - n2.y]))

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []

        for i in range(len(tree)):
            cur_node = tree[i]
            distance = LA.norm(np.array([cur_node.x - node.x, cur_node.y - node.y]))
            if distance <= self.neighbor_tolerance:
                neighborhood.append(cur_node)
                self.publish_point_marker(cur_node.x, cur_node.y, frame_id="laser", corlor_r=0.0, corlor_g=0.0, color_b=255.0)
        
        return neighborhood


    def choose_parent(self, tree, neighbors, new_node):
        min_cost = float('inf')
        best_parent = None
        for node in neighbors:
            if self.check_collision(node, new_node):
                c = self.cost(tree, node) + self.line_cost(node, new_node)
                if c < min_cost:
                    min_cost = c
                    best_parent = node
        if best_parent:
            new_node.parent = best_parent
            new_node.cost = min_cost
    
    def rewire(self, tree, neighbors, new_node):
        for node in neighbors:
            if node != new_node.parent and self.check_collision(node, new_node):
                c = self.cost(tree, new_node) + self.line_cost(new_node, node)
                if c < node.cost:
                    node.parent = new_node
                    node.cost = c

def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
