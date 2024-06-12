import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import re

class Color:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b
class VoronoiVisualizer(Node):
    def __init__(self):
        super().__init__('voronoi_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'voronoi_markers', 10)
        self.file_path = '/home/hydran00/pc2_ws/voro_cells.gnu'

        self.timer = self.create_timer(0.04, self.publish_voronoi_cells)
    
    def publish_voronoi_cells(self):
        marker_array = MarkerArray()
        marker_id = 0
        color = Color(1.0, 0.0, 0.0)
        with open(self.file_path, 'r') as file:
            points = []
            for line in file:
                line = line.strip()
                if line:
                    if line == "---":
                        color = Color(0.0, 1.0, 0.0)
                    coords = re.split(r'\s+', line)
                    if len(coords) != 3:
                        continue
                    point = (float(coords[0]), float(coords[1]), float(coords[2]))
                    points.append(point)
                else:
                    if points:
                        marker = Marker()
                        marker.header.frame_id = "world"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = ""
                        marker.id = marker_id
                        marker_id += 1
                        marker.type = Marker.LINE_STRIP
                        marker.action = Marker.ADD
                        marker.scale.x = 0.01
                        marker.color.a = 1.0
                        marker.color.r = color.r
                        marker.color.g = color.g
                        marker.color.b = color.b
                        
                        for point in points:
                            p = Point()
                            p.x, p.y, p.z = point
                            marker.points.append(p)
                        
                        # Close the loop of the cell
                        p = Point()
                        p.x, p.y, p.z = points[0]
                        marker.points.append(p)

                        marker_array.markers.append(marker)
                        points = []

        # # Publish the last marker if there are remaining points
        # if points:
        #     marker = Marker()
        #     marker.header.frame_id = "world"
        #     marker.header.stamp = self.get_clock().now().to_msg()
        #     marker.ns = ""
        #     marker.id = marker_id
        #     marker.type = Marker.LINE_STRIP
        #     marker.action = Marker.ADD
        #     marker.scale.x = 0.01
        #     marker.color.a = 1.0
        #     marker.color.r = 1.0
        #     marker.color.g = 0.0
        #     marker.color.b = 0.0
            
        #     for point in points:
        #         p = Point()
        #         p.x, p.y, p.z = point
        #         marker.points.append(p)
            
        #     # Close the loop of the cell
        #     p = Point()
        #     p.x, p.y, p.z = points[0]
        #     marker.points.append(p)

        #     marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
        self.get_logger().info('Published Voronoi cells')
def main(args=None):
    rclpy.init(args=args)
    voronoi_visualizer = VoronoiVisualizer()
    rclpy.spin(voronoi_visualizer)
    voronoi_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()