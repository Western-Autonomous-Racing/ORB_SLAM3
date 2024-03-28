import rclpy
from rclpy.node import Node
import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np

from sklearn import metrics
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2

# This reads points from a point cloud and computes the DBSCAN clustering
def process_map(bagPath):
    # Open the bag file
    bag = rosbag2_py.StorageOptions(uri=bagPath)

    reader = rosbag2_py.SequentialReader()
    reader.open(bag)

    # Get the topic information
    topics = reader.get_all_topics_and_types()
    print(topics)

    # Get the topic information for the point cloud
    topic_info = reader.get_topic_information('/points')
    print(topic_info)

    # Read the messages from the bag file
    messages = reader.read_messages()
    print(messages)

    # Get the points from the point cloud
    points = []
    for topic, msg, t in messages:
        if topic == '/points':
            points.append(msg.data)
    print(points)

    # Compute the DBSCAN clustering
    X = np.array(points)
    db = DBSCAN(eps=0.3, min_samples=10).fit(X)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print('Estimated number of clusters: %d' % n_clusters_)
    print('Estimated number of noise points: %d' % n_noise_)
    print("Silhouette Coefficient: %0.3f"
          % metrics.silhouette_score(X, labels))

    # Plot the clusters
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)

        xy = X[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor='k', markersize=14)

        xy = X[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor='k', markersize=6)

    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.show()

    # Close the bag file
    reader.close()

    

def main(args=None):
    rclpy.init(args=args)

    process_map('./test_data/2024-03-24-T13-47-36-aceb_raws_only/recording/recording_0.db3')

    rclpy.shutdown()

if __name__ == '__main__':
    main()




# # This reads points from a point cloud and computes the DBSCAN clustering

# # Subscriber to listen to the point cloud topic played from the bag file
# class Subscriber(Node):

#     def __init__(self):
#         super().__init__('pointcloud_subscriber')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/points',
#             self.listener_callback,
#             10)
#         self.subscription

#     def listener_callback(self, msg):
#         # self.get_logger().info('Received point cloud message')
    
# def main(args=None):
#     rclpy.init(args=args)

#     subscriber = Subscriber()

#     rclpy.spin(subscriber)

#     subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()