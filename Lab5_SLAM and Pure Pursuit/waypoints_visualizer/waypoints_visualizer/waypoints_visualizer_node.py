from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
import csv

class WaypointsVisualizer(Node):
    def __init__(self):
        print("visualizer")
        super().__init__('waypoints_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 1)
        relative_path = "/sim_ws/src/waypoints_visualizer/"
        fname = "srcwaypoints_final"
        filename = relative_path + fname + '.csv'
        self.path_points = []
        with open(filename, 'r') as csvfile:   
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                data_point = (float(row[0]), float(row[1]))
                self.path_points.append(data_point)
        self.visualize_waypoints()

    def visualize_waypoints(self):
        marker_array = MarkerArray()
        for point in self.path_points:
            x = point[0]
            y = point[1]
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 255.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker_array.markers.append(marker)

        for i, marker in enumerate(marker_array.markers):
            marker.id = i

        # print(marker_array)
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    waypoints_visualizer = WaypointsVisualizer()
    rclpy.spin(waypoints_visualizer)
    waypoints_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
