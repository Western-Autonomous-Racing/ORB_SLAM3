import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from typing import Tuple
import hdbscan

class MapProcessor(Node):
    def __init__(self):
        super().__init__('map_processor')

        self.declare_parameter("raw_map_topic", "/raw_map_points")
        self.declare_parameter("refined_map_topic", "/refined_map_2D")
        self.declare_parameter("timeout", 5)
        self.declare_parameter("min_z", 0.0)
        self.declare_parameter("max_z", 2.2)
        self.declare_parameter("min_cluster_size", 25)
        self.declare_parameter("min_samples", 3)
        self.declare_parameter("cluster_selection_epsilon", 0.8)
        self.declare_parameter("alpha", 1.0)

        if self.has_parameter("raw_map_topic"):
            self.raw_map_topic_ = self.get_parameter("raw_map_topic").value

        if self.has_parameter("refined_map_topic"):
            self.refined_map_topic_ = self.get_parameter("refined_map_topic").value

        if self.has_parameter("timeout"):
            self.timeout_ = self.get_parameter("timeout").value

        if self.has_parameter("min_z"):
            self.min_z_ = self.get_parameter("min_z").value

        if self.has_parameter("max_z"):
            self.max_z_ = self.get_parameter("max_z").value

        if self.has_parameter("min_cluster_size"):
            self.min_cluster_size_ = self.get_parameter("min_cluster_size").value

        if self.has_parameter("min_samples"):
            self.min_samples_ = self.get_parameter("min_samples").value

        if self.has_parameter("cluster_selection_epsilon"):
            self.cluster_selection_epsilon_ = self.get_parameter("cluster_selection_epsilon").value

        if self.has_parameter("alpha"):
            self.alpha_ = self.get_parameter("alpha").value

        self.raw_points_ = np.array([])
        self.processed_points_ = np.array([])     
        self.processed_points_2d_ = np.array([])
        self.combined_processed_points_ = np.array([])

        self.raw_map_sub_ = self.create_subscription(PointCloud2, self.raw_map_topic_, self.callback_raw, 10) 

        self.last_received_time = self.get_clock().now()

        # self.clustered_pub_ = self.create_publisher(PointCloud2, "/clustered_map_points", 10)
        self.refined_pub_2D_ = self.create_publisher(PointCloud2, self.refined_map_topic_, 10)
        # self.combined_clustered_pub_ = self.create_publisher(PointCloud2, "/combined_clustered_map", 10)
        
        # Create a timer that fires every second and checks if a message has been received
        # self.timer = self.create_timer(1.0, self.check_timeout)

        # self.publish_cluster_callback = self.create_timer(0.005, self.publish_clustered_map)
        self.publish_cluster_callback2d = self.create_timer(0.005, self.publish_clustered_map2d)
        # self.publish_combined_cluster_callback = self.create_timer(0.005, self.publish_combined_clustered_map)


    def callback_raw(self, msg: PointCloud2):    
        self.raw_points_ = self.pc2_to_np_arr(msg)    
        
    def pc2_to_np_arr(self, data: PointCloud2) -> np.ndarray:
        if data is not None:
            
            # Parse the PointCloud2 message
            gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(gen), dtype=np.float32)

            self.last_received_time = self.get_clock().now()

            return points
        
        return np.array([])
    
    def clip_points(self, points: np.ndarray) -> np.ndarray:
        # Clip points based on z values
        points = points[(points[:, 2] > self.min_z_) & (points[:, 2] < self.max_z_)]
        return points
    
    # def publish_clustered_map(self):
    #     # cluster points initially in 3D
    #     raw_3d = self.clip_points(self.raw_points_)
    #     cluster_labels = self.cluster_points(raw_3d)
    #     cluster_labels = np.float32(cluster_labels.reshape(-1, 1))
    #     self.processed_points_ = np.concatenate((raw_3d, cluster_labels), axis=1)
    #     self.processed_points_ = self.processed_points_[self.processed_points_[:,3] > -1.0]
    #     clustered_msg = self.define_cluster_msg(self.processed_points_)
    #     self.clustered_pub_.publish(clustered_msg)

    def publish_clustered_map2d(self):
        # cluster points initially in 2D
        if self.raw_points_.size == 0 or self.raw_points_.shape[0] == 0 or self.raw_points_.shape[1] != 3:
            return
        
        raw_2d = self.clip_points(self.raw_points_)
        raw_2d[:, 2] = 0.0
        # raw_2d = np.concatenate((raw_2d, np.zeros((raw_2d.shape[0], 1))), axis=1)
        
        cluster_labels = self.cluster_points(raw_2d)
        cluster_labels = np.float32(cluster_labels.reshape(-1, 1))
        self.processed_points_2d_ = np.concatenate((raw_2d, cluster_labels), axis=1)
        self.processed_points_2d_ = self.processed_points_2d_[self.processed_points_2d_[:,3] > -1.0]
        clustered_msg2d = self.define_cluster_msg(self.processed_points_2d_)
        self.refined_pub_2D_.publish(clustered_msg2d)

    # def publish_combined_clustered_map(self):
    #     # combine 3D and 2D clustered points
    #     self.combined_processed_points_ = np.concatenate((self.processed_points_, self.processed_points_2d_), axis=0)
    #     self.combined_processed_points_[:, 2] = 0.0
        
    #     clustered_msg_combined = self.define_cluster_msg(self.combined_processed_points_)
    #     self.combined_clustered_pub_.publish(clustered_msg_combined)
    
    def cluster_points(self, points: np.ndarray) -> np.ndarray:
        clusterer = hdbscan.HDBSCAN(min_cluster_size=self.min_cluster_size_, min_samples=self.min_samples_, cluster_selection_epsilon=self.cluster_selection_epsilon_, alpha=self.alpha_)
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
        point_cloud.header.frame_id = "odom"
        point_cloud.header.stamp = self.get_clock().now().to_msg()        

        point_cloud.fields = fields
        point_cloud.height = 1
        point_cloud.width = len(points)
        point_cloud.point_step = len(fields) * 4
        point_cloud.is_bigendian = False
        total_points = point_cloud.width * point_cloud.height
        point_cloud.row_step = total_points * point_cloud.point_step
        point_cloud.is_dense = True

        return point_cloud     

    def check_timeout(self):
        # Check if the time since the last received message is greater than the timeout
        # if (self.get_clock().now() - self.last_received_time).seconds > self.timeout:
        pass        

    def get_map_points(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.raw_points_, self.processed_points_

            
def main():
    try:
        rclpy.init()
        recvMapNode = MapProcessor()
        rclpy.spin(recvMapNode)
    except KeyboardInterrupt:
        recvMapNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()