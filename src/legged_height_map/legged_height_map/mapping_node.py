import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import struct

def fit_plane_ransac(pts, dist_thresh=0.05, max_iters=100):
    """Pure NumPy RANSAC with strict outlier rejection."""
    best_inliers = np.zeros(len(pts), dtype=bool)
    best_normal = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    
    if len(pts) < 3:
        return best_normal, best_inliers
        
    for _ in range(max_iters):
        idx = np.random.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[idx]
        
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        
        if norm < 1e-6:
            continue
            
        normal = normal / norm
        d = -np.dot(normal, p0)
        dists = np.abs(np.dot(pts, normal) + d)
        
        inliers = dists < dist_thresh
        if np.sum(inliers) > np.sum(best_inliers):
            best_inliers = inliers
            best_normal = normal
            
    return best_normal, best_inliers

class FOGMappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        # 1. Core Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_cloud_topic', '/cloud_registered'),
                ('odom_topic', '/Odometry'),
                ('grid_resolution', 0.05),
                ('map_size', 4.0),
                ('num_layers', 60),
                ('layer_height', 0.05),
                ('p_occ_threshold', 0.55),
                ('l_occ', 0.85),
                ('l_free', -0.40),
                ('floor_gate_epsilon', 0.08),
                ('floor_smoothing_nu', 0.7),
                ('plane_dist_threshold', 0.05),
                ('plane_max_count', 6),
                ('plane_normal_angle_deg', 45.0) # Wide tolerance for FAST-LIO tilt
            ]
        )
        
        self.res = float(self.get_parameter('grid_resolution').value)
        self.map_size = float(self.get_parameter('map_size').value)
        self.grid_cells = int(self.map_size / self.res)
        self.num_layers = int(self.get_parameter('num_layers').value)
        self.layer_height = float(self.get_parameter('layer_height').value)
        
        cloud_topic = self.get_parameter('input_cloud_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        # 2. Subscribers & Publishers
        self.sub_cloud = self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.pub_fog = self.create_publisher(PointCloud2, '/legged_height_map/fog_cloud', 10)
        
        # 3. 3D & 2D Grids
        self.occupancy_grid = np.zeros((self.grid_cells, self.grid_cells, self.num_layers), dtype=np.float32)
        self.floor_height = np.full((self.grid_cells, self.grid_cells), np.nan, dtype=np.float32)
        self.obstacle_height = np.full((self.grid_cells, self.grid_cells), np.nan, dtype=np.float32)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.prev_x = None
        self.prev_y = None

        self.get_logger().info("FOG Mapping Node Initialized (True Ray Tracing Active).")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z

        if self.prev_x is None:
            self.prev_x = self.robot_x
            self.prev_y = self.robot_y
            return

        dx = self.robot_x - self.prev_x
        dy = self.robot_y - self.prev_y
        
        shift_x = int(np.floor(dx / self.res))
        shift_y = int(np.floor(dy / self.res))
        
        if shift_x != 0 or shift_y != 0:
            self.occupancy_grid = np.roll(self.occupancy_grid, shift=(-shift_x, -shift_y), axis=(0, 1))
            self.floor_height = np.roll(self.floor_height, shift=(-shift_x, -shift_y), axis=(0, 1))
            self.obstacle_height = np.roll(self.obstacle_height, shift=(-shift_x, -shift_y), axis=(0, 1))
            
            if shift_x > 0:
                self.occupancy_grid[-shift_x:, :, :] = 0
                self.floor_height[-shift_x:, :] = np.nan
                self.obstacle_height[-shift_x:, :] = np.nan
            elif shift_x < 0:
                self.occupancy_grid[:-shift_x, :, :] = 0
                self.floor_height[:-shift_x, :] = np.nan
                self.obstacle_height[:-shift_x, :] = np.nan
                
            if shift_y > 0:
                self.occupancy_grid[:, -shift_y:, :] = 0
                self.floor_height[:, -shift_y:] = np.nan
                self.obstacle_height[:, -shift_y:] = np.nan
            elif shift_y < 0:
                self.occupancy_grid[:, :-shift_y, :] = 0
                self.floor_height[:, :-shift_y] = np.nan
                self.obstacle_height[:, :-shift_y] = np.nan
                
            self.prev_x += shift_x * self.res
            self.prev_y += shift_y * self.res

    def cloud_callback(self, msg):
        # 1. Parse Humble Structured Array
        cloud_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(cloud_data) < 30: return
            
        points = np.empty((len(cloud_data), 3), dtype=np.float32)
        points[:, 0] = cloud_data['x']
        points[:, 1] = cloud_data['y']
        points[:, 2] = cloud_data['z']

        half_size = self.map_size / 2.0
        
        # 2. Strict Bounding Box, Body Filter, & CEILING REMOVAL
        rel_x = points[:, 0] - self.robot_x
        rel_y = points[:, 1] - self.robot_y
        rel_z = points[:, 2] - self.robot_z  # Calculate height relative to robot
        dist_2d = np.sqrt(rel_x**2 + rel_y**2)
        
        # Keep points within 4x4m, ignore robot body (<0.25m), and IGNORE CEILINGS (>0.5m above robot)
        mask = (np.abs(rel_x) < half_size) & (np.abs(rel_y) < half_size) & (dist_2d > 0.25) & (rel_z < 0.5)
        points = points[mask]
        
        if len(points) < 30: return

        # 3. Iterative RANSAC Segmenter
        floor_points, obstacle_points = [], []
        rem_points = points.copy()
        
        max_angle_rad = np.radians(self.get_parameter('plane_normal_angle_deg').value)
        dist_thresh = self.get_parameter('plane_dist_threshold').value
        max_planes = self.get_parameter('plane_max_count').value

        for _ in range(max_planes):
            if len(rem_points) < 30: break
                
            normal, inliers = fit_plane_ransac(rem_points, dist_thresh=dist_thresh, max_iters=100)
            if np.sum(inliers) < 20: break
                
            angle_with_z = np.arccos(np.clip(np.abs(normal[2]), -1.0, 1.0))
            if angle_with_z > np.pi / 2.0: angle_with_z = np.pi - angle_with_z
                
            if angle_with_z < max_angle_rad:
                floor_points.append(rem_points[inliers])
            else:
                obstacle_points.append(rem_points[inliers])
                
            rem_points = rem_points[~inliers]

        if len(rem_points) > 0: obstacle_points.append(rem_points)

        floor_pts = np.vstack(floor_points) if floor_points else np.empty((0, 3), dtype=np.float32)
        obs_pts = np.vstack(obstacle_points) if obstacle_points else np.empty((0, 3), dtype=np.float32)

        # 4. Discretization Function
        def points_to_grid(pts):
            if len(pts) == 0: return np.empty((0, 3), dtype=int)
            idx_xy = (((pts[:, :2] - [self.robot_x, self.robot_y]) + half_size) / self.res).astype(int)
            idx_z = (((pts[:, 2] - self.robot_z) + (self.num_layers * self.layer_height / 2.0)) / self.layer_height).astype(int)
            grid_idx = np.column_stack((idx_xy, idx_z))
            np.clip(grid_idx[:, 0], 0, self.grid_cells - 1, out=grid_idx[:, 0])
            np.clip(grid_idx[:, 1], 0, self.grid_cells - 1, out=grid_idx[:, 1])
            np.clip(grid_idx[:, 2], 0, self.num_layers - 1, out=grid_idx[:, 2])
            return grid_idx

        # 5. True Fractional Ray Tracing (Gutmann Eq. 6)
        # Generates negative evidence along the line of sight to erase ghost obstacles
        sensor_origin = np.array([self.robot_x, self.robot_y, self.robot_z + 0.15], dtype=np.float32)
        ray_fractions = np.array([0.33, 0.66, 0.85], dtype=np.float32)
        
        ray_vectors = points - sensor_origin
        free_pts = np.vstack([sensor_origin + f * ray_vectors for f in ray_fractions])
        
        free_idx = points_to_grid(free_pts)
        floor_idx = points_to_grid(floor_pts)
        obs_idx = points_to_grid(obs_pts)

        l_occ = self.get_parameter('l_occ').value
        l_free = self.get_parameter('l_free').value

        # Apply probabilities
        if len(free_idx) > 0: self.occupancy_grid[free_idx[:,0], free_idx[:,1], free_idx[:,2]] += l_free
        if len(floor_idx) > 0: self.occupancy_grid[floor_idx[:,0], floor_idx[:,1], floor_idx[:,2]] += l_occ
        if len(obs_idx) > 0: self.occupancy_grid[obs_idx[:,0], obs_idx[:,1], obs_idx[:,2]] += l_occ

        self.occupancy_grid = np.clip(self.occupancy_grid, -2.0, 3.5)

        # 6. Extraction (Gutmann 7-10)
        p_occ_thresh = self.get_parameter('p_occ_threshold').value
        l_thresh = np.log(p_occ_thresh / (1.0 - p_occ_thresh)) if p_occ_thresh < 1.0 else 3.4
        epsilon = self.get_parameter('floor_gate_epsilon').value
        
        occ_mask = self.occupancy_grid > l_thresh
        any_occ = np.any(occ_mask, axis=2)

        z_coords = np.arange(self.num_layers) * self.layer_height - (self.num_layers * self.layer_height / 2.0) + self.robot_z
        highest_occ_idx = self.num_layers - 1 - np.argmax(occ_mask[:, :, ::-1], axis=2)
        
        self.obstacle_height[:] = np.nan
        self.obstacle_height[any_occ] = z_coords[highest_occ_idx[any_occ]]

        if len(floor_idx) > 0:
            valid_floor = occ_mask[floor_idx[:,0], floor_idx[:,1], floor_idx[:,2]]
            for i, (xi, yi, _) in enumerate(floor_idx[valid_floor]):
                self.floor_height[xi, yi] = floor_pts[valid_floor][i, 2]

        self.floor_height[~any_occ] = np.nan

        # 7. Visualization & Quantization Correction
        fog_points = []
        # Expand visual gate to account for grid snapping so floor cells aren't painted red
        vis_epsilon = epsilon + self.layer_height 

        for x in range(self.grid_cells):
            for y in range(self.grid_cells):
                if not any_occ[x, y]: continue
                
                wx = x * self.res - half_size + self.robot_x
                wy = y * self.res - half_size + self.robot_y
                f_z = self.floor_height[x, y]
                o_z = self.obstacle_height[x, y]
                
                if not np.isnan(f_z):
                    if not np.isnan(o_z) and (o_z - f_z) > vis_epsilon:
                        # Legitimate Obstacle (RED: B=0, G=0, R=255)
                        rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
                        fog_points.append([wx, wy, o_z, rgb])
                    else:
                        # Ground Floor Support (GREEN: B=0, G=255, R=0)
                        rgb = struct.unpack('I', struct.pack('BBBB', 0, 255, 0, 255))[0]
                        fog_points.append([wx, wy, f_z, rgb])
                elif not np.isnan(o_z):
                    # Unclassified Obstacle (RED: B=0, G=0, R=255)
                    rgb = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
                    fog_points.append([wx, wy, o_z, rgb])

        if fog_points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            self.pub_fog.publish(pc2.create_cloud(header, fields, fog_points))

def main(args=None):
    rclpy.init(args=args)
    node = FOGMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()