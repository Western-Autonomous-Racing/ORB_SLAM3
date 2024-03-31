import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from typing import Tuple
import hdbscan
import os
import cv2
import yaml

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
        self.declare_parameter("enable_save", True)
        self.declare_parameter("map_dir", "../output_map")
        self.declare_parameter("map_name", "map")
        self.declare_parameter("resolution", 0.1) # metres per pixel
        self.declare_parameter("map_padding", 1) # metres

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

        if self.has_parameter("enable_save"):
            self.enable_save_ = self.get_parameter("enable_save").value
            if self.enable_save_ and self.has_parameter("map_name") and self.has_parameter("map_dir") and self.has_parameter("resolution"):
                self.map_name_ = self.get_parameter("map_name").value
                self.map_dir_ = self.get_parameter("map_dir").value
                self.m_per_pixel_ = self.get_parameter("resolution").value

                if self.has_parameter("map_padding"):
                    self.map_padding_ = self.get_parameter("map_padding").value

        self.raw_points_ = np.array([])
        self.processed_points_2d_ = np.array([])

        self.raw_map_sub_ = self.create_subscription(PointCloud2, self.raw_map_topic_, self.callback_raw, 10) 

        self.last_received_time = self.get_clock().now()

        self.refined_pub_2D_ = self.create_publisher(PointCloud2, self.refined_map_topic_, 10)

        self.publish_cluster_callback2d = self.create_timer(0.005, self.publish_clustered_map2d)

    def save_map(self):
        if self.processed_points_2d_.size == 0 or self.processed_points_2d_.shape[0] == 0 or self.processed_points_2d_.shape[1] != 4:
            return
        
        points= self.processed_points_2d_[:, :3]

        # get x maxes, x mins, y maxes, y mins
        x_max = np.max(points[:, 0])
        x_min = np.min(points[:, 0])
        y_max = np.max(points[:, 1])
        y_min = np.min(points[:, 1])

        x_range = abs(x_max - x_min)
        y_range = abs(y_max - y_min)

        x_res = int(x_range / self.m_per_pixel_)
        y_res = int(y_range / self.m_per_pixel_) 
        x_padding = int(self.map_padding_ / self.m_per_pixel_)
        y_padding = int(self.map_padding_ / self.m_per_pixel_)
        x_res += 2 * x_padding
        y_res += 2 * y_padding

        # create a blank image
        map_image = np.full((y_res, x_res), 255, dtype=np.uint8)

    

        # scale points to image size
        points[:, 0] = ((points[:, 0] - x_min) * (x_res / x_range)) + x_padding + (x_res/2)
        points[:, 1] = ((points[:, 1] - y_min) * (y_res / y_range)) + y_padding + (y_res/2)

        points = np.round(points).astype(np.uint8)

        for point in points:
            map_image[point[1], point[0]] = 0

        # draw points on image
        # print(points.shape)
        # print(map_image)
        
        cv2.imshow("map", map_image)
        cv2.waitKey(0)

        # if not os.path.exists(self.map_dir_):
        #     os.makedirs(self.map_dir_)

        # map_file = os.path.join(self.map_name_ + ".png")

        # map_image = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

        # ret = cv2.imwrite(map_file, map_image)

        # if not ret:
        #     self.get_logger().error("Failed to save map")

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
    
    def publish_clustered_map2d(self):
        # cluster points initially in 2D
        if self.raw_points_.size == 0 or self.raw_points_.shape[0] == 0 or self.raw_points_.shape[1] != 3:
            return
        
        raw_2d = self.clip_points(self.raw_points_)
        raw_2d[:, 2] = 0.0
        
        cluster_labels = self.cluster_points(raw_2d)
        cluster_labels = np.float32(cluster_labels.reshape(-1, 1))
        self.processed_points_2d_ = np.concatenate((raw_2d, cluster_labels), axis=1)
        self.processed_points_2d_ = self.processed_points_2d_[self.processed_points_2d_[:,3] > -1.0]
        clustered_msg2d = self.define_cluster_msg(self.processed_points_2d_)
        self.refined_pub_2D_.publish(clustered_msg2d)
    
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

    def get_ts(self, timestamp: Time) -> float:
        seconds, nanoseconds = timestamp.seconds_nanoseconds()
        return seconds + nanoseconds * 1e-9  
    
    def msg_timeout(self) -> bool:
        current_time = self.get_clock().now()
        diff = self.get_ts(current_time) - self.get_ts(self.last_received_time)
        if diff > self.timeout_:
            return True
        return False
            
def main():
    try:
        rclpy.init()
        recvMapNode = MapProcessor()
        rclpy.spin(recvMapNode)
    except KeyboardInterrupt:
        recvMapNode.save_map()
        recvMapNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()