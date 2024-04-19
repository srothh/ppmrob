import numpy as np

class OnlineKMeans:
    def __init__(self, n_clusters, min_dist, init_points=None):
        self.n_clusters = n_clusters
        self.min_dist = min_dist
        self.cluster_centers = self.__initialize_centers(init_points)
        self.counts = np.ones(n_clusters)  # Starting with 1 to avoid division by zero

    def __initialize_centers(self, init_points):
        if init_points is not None:
            return np.array(init_points)
        centers = []
        while len(centers) < self.n_clusters:
            if len(centers) == 0:
                # Randomly select the first center
                new_center = np.random.rand(2)  # Assuming 2D, adjust dimensions as necessary
            else:
                # Ensure new center is at least min_dist away from all existing centers
                while True:
                    new_center = np.random.rand(2)
                    if all(np.linalg.norm(new_center - c) >= self.min_dist for c in centers):
                        break
            centers.append(new_center)
        return np.array(centers)

    def add_point(self, point):
        # Find the nearest cluster center
        distances = np.linalg.norm(self.cluster_centers - point, axis=1)
        nearest_cluster = np.argmin(distances)

        # Update the cluster center
        self.cluster_centers[nearest_cluster] += (point - self.cluster_centers[nearest_cluster]) / self.counts[nearest_cluster]
        self.counts[nearest_cluster] += 1