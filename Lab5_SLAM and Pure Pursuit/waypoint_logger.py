#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion  # sudo apt install ros-foxy-tf-transformations 
import time


#relative_path = "/home/nvidia/f1tenth_ws/src/pure_pursuit/pure_pursuit/waypoints/"
relative_path = "/home/tj/sim_ws/src"
fname = "srcwaypoints_final"
print(relative_path+fname+'.csv')
# file = open(strftime(relative_path+'waypoint-%Y-%m-%d-%H-%M-%S', gmtime())+'.csv', 'w')
file = open(relative_path+fname+'.csv', 'w')


previous_time = 0.0

class WaypointLogger(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom',self.save_waypoint, 10)
        #self.subscriptions  # prevent unused variable warning


    def save_waypoint(self,  data):
        global previous_time
        collection_time = 0.5
        if(data.header.stamp.sec - previous_time >= collection_time):
            print("next point", data.header.stamp.sec)
            quaternion = np.array([data.pose.pose.orientation.x,
                                data.pose.pose.orientation.y,
                                data.pose.pose.orientation.z,
                                data.pose.pose.orientation.w])

            euler = euler_from_quaternion(quaternion)
            speed = 3.0
            lookahead = 1.0

            file.write('%f,%f,%f,%f,%f,%f\n' % (data.pose.pose.position.x,
                                            data.pose.pose.position.y,
                                            euler[2],
                                            speed,
                                            lookahead,
                                            data.header.stamp.sec))

            previous_time = data.header.stamp.sec
 
def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    print('Saving waypoints...')
    main()
