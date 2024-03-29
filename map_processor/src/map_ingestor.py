import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from typing import Tuple
import hdbscan
import open3d

class MapReceiver(Node):
    def __init__(self):
        super().__init__('map_recv')

        self.declare_parameter("raw_map_topic", "/raw_map_points")
        self.declare_parameter("refined_map_topic", "/refined_map_points")
        self.declare_parameter("timeout", 5)
        self.declare_parameter("min_cluster_size", 50)
        self.declare_parameter("enable_visualization", False)


        if self.has_parameter("raw_map_topic"):
            self.raw_map_topic = self.get_parameter("raw_map_topic").value

        if self.has_parameter("refined_map_topic"):
            self.refined_map_topic = self.get_parameter("refined_map_topic").value

        if self.has_parameter("timeout"):
            self.timeout = self.get_parameter("timeout").value

        if self.has_parameter("min_cluster_size"):
            self.min_cluster_size = self.get_parameter("min_cluster_size").value

        self.raw_points = np.array([])
        self.refined_points = np.array([])     

        self.raw_map_sub_ = self.create_subscription(PointCloud2, self.raw_map_topic, self.callback_raw, 10) 
        self.refined_map_sub_ = self.create_subscription(PointCloud2, self.refined_map_topic, self.callback_refined, 10)
        self.clustered_pub_ = self.create_publisher(PointCloud2, "/clustered_map", 10)
        
        self.last_received_time = self.get_clock().now()

        # Create a timer that fires every second and checks if a message has been received
        # self.timer = self.create_timer(1.0, self.check_timeout)

        self.publish_cluster_callback = self.create_timer(0.02, self.publish_clustered_map)

    def callback_raw(self, msg: PointCloud2):    
        self.raw_points = self.pc2_to_np_arr(msg)   

    def callback_refined(self, msg: PointCloud2):    
        self.refined_points = self.pc2_to_np_arr(msg)   
        
    def pc2_to_np_arr(self, data: PointCloud2) -> np.ndarray:
        if data is not None:
            
            # Parse the PointCloud2 message
            gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            points_numpy = np.array(list(gen), dtype=np.float32)

            self.last_received_time = self.get_clock().now()

            return points_numpy
        
        return np.array([])
    
    def publish_clustered_map(self):
        if len(self.refined_points) > 0:
            cluster_labels = self.cluster_points(self.refined_points)
            clustered_points = np.concatenate((self.refined_points, np.float32(cluster_labels.reshape(-1, 1))), axis=1)
            clustered_msg = self.define_cluster_msg(clustered_points)
            self.clustered_pub_.publish(clustered_msg)
    
    def cluster_points(self, points: np.ndarray) -> np.ndarray:
        clusterer = hdbscan.HDBSCAN(min_cluster_size=10)
        cluster_labels = clusterer.fit_predict(points)

        return cluster_labels


    def define_cluster_msg(self, points: np.ndarray) -> PointCloud2:
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        point_cloud = pc2.create_cloud(header=Header(), fields=fields, points=points)
        point_cloud.header.frame_id = "map"
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        

        point_cloud.fields = fields

        point_cloud.height = 1
        point_cloud.width = len(points)


        point_cloud.point_step = len(fields) * 4

        total_points = point_cloud.width * point_cloud.height
        point_cloud.row_step = total_points * point_cloud.point_step
        point_cloud.is_dense = True

        return point_cloud    


    def publish_clustered_map(self):
        if len(self.refined_points) > 0:
            cluster_labels = self.cluster_points(self.refined_points)
            clustered_points = np.concatenate((self.refined_points, np.float32(cluster_labels.reshape(-1, 1))), axis=1)
            clustered_msg = self.define_cluster_msg(clustered_points)
            self.clustered_pub_.publish(clustered_msg)

    def check_timeout(self):
        # Check if the time since the last received message is greater than the timeout
        # if (self.get_clock().now() - self.last_received_time).seconds > self.timeout:
        pass        

    def get_map_points(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.raw_points, self.refined_points     

            
def main():
    rclpy.init()
    recvMapNode = MapReceiver()
    rclpy.spin(recvMapNode)
    print(recvMapNode.get_map_points())
    recvMapNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()