import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import os

class PCDSaverNode(Node):
    def __init__(self):
        super().__init__('pcd_saver_node')
        self.sub = self.create_subscription(PointCloud2, '/legged_height_map/fog_cloud', self.cloud_cb, 10)
        self.latest_cloud = None
        self.get_logger().info("PCD Saver active. Play your bag, then press Ctrl+C here to save the map!")

    def cloud_cb(self, msg):
        self.latest_cloud = msg

    def save_pcd(self):
        if self.latest_cloud is None:
            self.get_logger().info("No point cloud data received. Nothing to save.")
            return
            
        points = list(pc2.read_points(self.latest_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        if not points:
            return

        path = os.path.expanduser('~/Downloads/final_fog_map.pcd')
        
        with open(path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\nDATA ascii\n")
            for p in points:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {p[3]}\n")
        
        self.get_logger().info(f"SUCCESS: Saved {len(points)} map points to {path}")

def main():
    rclpy.init()
    node = PCDSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_pcd()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
