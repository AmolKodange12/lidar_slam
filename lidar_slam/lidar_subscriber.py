import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry.laser_geometry import LaserProjection
from geometry_msgs.msg import TransformStamped
import matplotlib.pyplot as plt
import tf2_ros

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.laser_projector = LaserProjection()
        #Subscription to the /laser_scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.listener_callback,
            10)
        #Publishing the pcl-converted msg to the /laser_pointcloud topic 
        self.publisher = self.create_publisher(
            PointCloud2,
            '/laser_pointcloud',
            10)
        
        #Initialize the TransformBroadcaster and variable to store the accumulated point cloud
        self.br = tf2_ros.TransformBroadcaster(self)
        self.map_cloud = None
            
        # Initialize real-time plotting
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.sc = self.ax.scatter([], [], c='b', marker='.')
        self.ax.set_xlim(-8, 8)  # Set X limits
        self.ax.set_ylim(-8, 8)  # Set Y limits
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Real-Time Lidar Scan (Top View)')

    def listener_callback(self, msg):
        #self.get_logger().info('Received Lidar scan data')
        angle_increment = msg.angle_increment
        current_angle = msg.angle_min
        x_points = []
        y_points = []
        for range_value in msg.ranges:
            if np.isfinite(range_value) and range_value > 0.1:
                # Normalize angle within [-π, π] range
                normalized_angle = np.arctan2(np.sin(current_angle), np.cos(current_angle))
                
                # Convert polar to Cartesian (rotated 90°)
                x = -range_value * np.sin(normalized_angle)  
                y = range_value * np.cos(normalized_angle)  
                x_points.append(x)
                y_points.append(y)
            current_angle += angle_increment
        # Update scatter plot
        self.sc.set_offsets(np.c_[x_points, y_points])  # Update (X, Y) points
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)
        # Publish PointCloud2 message
        cloud_msg = self.laser_projector.projectLaser(msg)
        cloud_msg.header.frame_id = "map"
        self.publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    lidar_subscriber.destroy_node()
    rclpy.shutdown()
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep the final plot open

if __name__ == '__main__':
    main()

