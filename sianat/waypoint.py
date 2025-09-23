import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class MinimalNode(Node):
    def __init__(self):
        super().__init__('waypoint_selection')

        self.create_subscription(Path, '/sianat/raw_path', self.path_cb, 1)
        self.create_subscription(PoseStamped, '/aruco/pose', self.pose_cb, 1)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/sianat/waypoint', 1)
        self.points = np.array([])
        self.position = (0, 0, 0)
    
    def path_cb(self, msg: Path):
        poses = msg.poses
        self.points = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses])

    def pose_cb(self, msg: PoseStamped):
        self.position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()