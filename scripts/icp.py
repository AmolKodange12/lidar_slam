import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import tf2_ros
import copy
import scipy.spatial.transform as sst
class ICPMapping(Node):
    def __init__(self):
        super().__init__('icp_mapping')

        self.cloud_sub = self.create_subscription(
            PointCloud2, '/laser_pointcloud', self.cloud_callback, 10)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.map_cloud = None  # Stores the accumulated point cloud
        self.global_transform = np.identity(4)  # Global transformation matrix

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
        print(cloud_xyz)
        print(cloud_xyz.dtype)
        if cloud_xyz.ndim == 1:  # In case it's a 1D array of structured data
            cloud_xyz = cloud_xyz.reshape(-1, 3)
        print(cloud_xyz.shape)
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
            self.map_cloud += transformed_cloud

            #self.map_cloud.transform(reg_p2p.transformation)
            #self.map_cloud += new_cloud

            self.broadcast_tf(reg_p2p.transformation)
        
        self.vis.update_geometry(self.vis_geometry)
        self.vis.poll_events()
        self.vis.update_renderer()

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

def main(args=None):
    rclpy.init(args=args)
    node = ICPMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
