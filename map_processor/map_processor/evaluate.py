import hdbscan
from sklearn.datasets import make_blobs
import matplotlib.pyplot as plt

data, _ = make_blobs(1000,3)

clusterer = hdbscan.HDBSCAN(min_cluster_size=10)
cluster_labels = clusterer.fit_predict(data)

print(cluster_labels)
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2], c=cluster_labels)
plt.show()
