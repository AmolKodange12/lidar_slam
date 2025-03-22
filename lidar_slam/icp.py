import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import tf2_ros
import copy
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation
import scipy.spatial.transform as sst
class ICPMapping(Node):
    def __init__(self):
        super().__init__('icp_mapping')

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/laser_pointcloud', self.cloud_callback, 10)
        
        self.scan_matching_sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self.scan_matching_callback, 10)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.map_cloud = None  # Stores the accumulated point cloud
        self.global_transform = np.identity(4)  # Global transformation matrix
        self.previous_clouds = []  # Stores past clouds for loop closure
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Accumulated Map")
        self.vis_geometry = None  # Placeholder for real-time update
    def cloud_callback(self, msg):
        cloud = []
        for c in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            cloud.append(c)
        if len(cloud) <5:
            return
        cloud = np.array(cloud)
        
        #cloud_xyz = cloud[['x', 'y', 'z']]  # Extract x, y, z fields
        if np.size(cloud) == 0:
            return
        cloud_xyz = cloud.view(np.float32).reshape(cloud.shape + (-1,))
        if cloud_xyz.ndim == 1:  # In case it's a 1D array of structured data
            cloud_xyz = cloud_xyz.reshape(-1, 3)
        new_cloud = o3d.geometry.PointCloud()
        new_cloud.points = o3d.utility.Vector3dVector(cloud_xyz)

        if self.map_cloud is None:
            self.map_cloud = new_cloud
            self.vis_geometry = self.map_cloud  # Set the initial geometry
            self.vis.add_geometry(self.vis_geometry)
        else:
            if hasattr(self, "last_transformation"):
                initial_guess = self.last_transformation
            else:
                # Use the global transformation as the initial guess for ICP
                initial_guess = self.global_transform
            # Perform ICP registration
            reg_p2p = o3d.pipelines.registration.registration_icp(
                new_cloud, self.map_cloud, 0.05,
                initial_guess,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            transformed_cloud = copy.deepcopy(new_cloud)
            transformed_cloud.transform(reg_p2p.transformation)
            # Store transformed clouds for loop closure detection
            self.previous_clouds.append((transformed_cloud, reg_p2p.transformation))
            
            # Detect loop closure and correct map
            corrected_transformation = self.detect_loop_closure(transformed_cloud)
            transformed_cloud.transform(corrected_transformation)
            self.map_cloud += transformed_cloud
            self.broadcast_tf(corrected_transformation)
        
        self.vis.update_geometry(self.vis_geometry)
        self.vis.poll_events()
        self.vis.update_renderer()

    def detect_loop_closure(self, current_cloud):
        """
        Detects if the robot has revisited a previously seen place and corrects the drift.
        """
        if len(self.previous_clouds) < 3:
            return np.identity(4)  # Not enough past scans for loop closure

        best_match = None
        min_distance = float('inf')
        
        current_fpfh = self.compute_fpfh_features(current_cloud)
        
        for past_cloud, past_transform in self.previous_clouds[:-2]:  # Skip last two for smoother transition
            past_fpfh = self.compute_fpfh_features(past_cloud)
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                current_cloud, past_cloud, current_fpfh, past_fpfh,
                mutual_filter=False,  # ✅ Corrected argument placement
                max_correspondence_distance=0.05,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                ransac_n=4,
                checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.05)],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(50000, 0.999)
            )
            
            fitness = result.fitness
            if fitness > 0.6 and fitness < min_distance:
                min_distance = fitness
                best_match = result.transformation
        
        if best_match is not None:
            return best_match  # Use corrected transformation
        return np.identity(4)

    def broadcast_tf(self, transformation_matrix):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "lidar"
        t.transform.translation.x = transformation_matrix[0, 3]
        t.transform.translation.y = transformation_matrix[1, 3]
        t.transform.translation.z = transformation_matrix[2, 3]

        # Convert rotation matrix to quaternion
        rotation = sst.Rotation.from_matrix(np.array(transformation_matrix[:3, :3], copy=True)).as_quat()
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.br.sendTransform(t)
    
    def compute_fpfh_features(self, pcd):
        """
        Computes FPFH features for loop closure detection.
        """
        radius_normal = 0.1
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=100)
        )
        return fpfh

    def broadcast_tf(self, transformation_matrix):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "lidar"
        t.transform.translation.x = transformation_matrix[0, 3]
        t.transform.translation.y = transformation_matrix[1, 3]
        t.transform.translation.z = transformation_matrix[2, 3]

        rotation = sst.Rotation.from_matrix(np.array(transformation_matrix[:3, :3], copy=True)).as_quat()
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.br.sendTransform(t)

    def scan_matching_callback(self, msg):
        """
        Callback for the odometry topic. Updates the global transformation matrix.
        """
        # Extract the position and orientation from the message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert orientation to Euler angles
        euler = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        theta = euler.as_matrix()[2]  # Yaw angle

        # If this is the first message, initialize previous position and orientation
        if not hasattr(self, 'previous_position'):
            self.previous_position = position
            self.previous_theta = theta
            return

        # Calculate the difference in position and orientation
        dx = position.x - self.previous_position.x
        dy = position.y - self.previous_position.y
        d_theta = theta - self.previous_theta

        # Update the previous position and orientation
        self.previous_position = position
        self.previous_theta = theta

        # Print the differences
        self.get_logger().info(f'dx: {dx}, dy: {dy}, d_theta: {d_theta}')
def main(args=None):
    rclpy.init(args=args)
    node = ICPMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
