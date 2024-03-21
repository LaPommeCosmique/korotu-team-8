#!/usr/bin/env python

# Import the necessary ROS libraries
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import numpy as np
from sklearn.cluster import DBSCAN

class ObstacleDetector:
    def __init__(self):
        # Initializes the ROS node
        rospy.init_node('obstacle_detector', anonymous=True)
        
        # Publisher which will publish to the topic '/obstacle_radius'.
        self.radius_publisher = rospy.Publisher('/obstacle_radius', Float32, queue_size=10)
        
        # Subscriber which will listen to the topic '/map'.
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Setting the rate to sleep the loop at.
        # Used to control the frequency at which your loop operates
        self.rate = rospy.Rate(10)  # 10hz

    def map_callback(self, data):
        width = data.info.width
        height = data.info.height
        resolution = data.info.resolution
        origin = data.info.origin.position

        obstacle_points = []
        for y in range(height):
            for x in range(width):
                if data.data[y * width + x] >= 50:  # Threshold for obstacles
                    real_x = origin.x + (x + 0.5) * resolution
                    real_y = origin.y + (y + 0.5) * resolution
                    obstacle_points.append([real_x, real_y])
        
        clustering = DBSCAN(eps=resolution*1.5, min_samples=3).fit(obstacle_points)
        labels = clustering.labels_
        
        clusters = {}
        for label, point in zip(labels, obstacle_points):
            if label == -1:
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(point)
        
        for label, points in clusters.items():
            points = np.array(points)
            centroid = points.mean(axis=0)
            radius = np.max(np.sqrt(np.sum((points - centroid)**2, axis=1)))
            
            rospy.loginfo(f"Obstacle {label}: Centroid = {centroid}, Radius = {radius} meters")
            
            # Publishing the radius value to the /obstacle_radius topic
            self.radius_publisher.publish(Float32(radius))

    def run(self):
        # Keep the program running until shutdown signal is received
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    # Create the object of the class ObstacleDetector
    obstacle_detector = ObstacleDetector()
    # Run the object to keep the node alive
    obstacle_detector.run()
