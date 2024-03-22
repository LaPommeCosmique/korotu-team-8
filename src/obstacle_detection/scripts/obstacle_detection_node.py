#!/usr/bin/env python

# Import the necessary ROS libraries
import rospy
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import numpy as np
from sklearn.cluster import DBSCAN

class ObstacleDetector:
    def __init__(self):
        # Initializes the ROS node
        rospy.init_node('obstacle_detector', anonymous=True)
        
        # Publisher which will publish to the topic '/obstacle_radius'.
        self.obstacle_radius_publisher = rospy.Publisher('/obstacle_radius', Float32, queue_size=10)
        # Publisher which will publish to the topic '/obstacle_coordinates'.
        self.obstacle_coordinate_publisher = rospy.Publisher('/obstacle_coordinates', Float32, queue_size=10)
        # Publisher which will publish to the topic '/obstacle_present'.
        self.obstacle_present_publisher = rospy.Publisher('/obstacle_present', Boolean, queue_size=10)


        # Subscriber which will listen to the topic '/map'.
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.obstacle_detection_callback)
        
        # Setting the rate to sleep the loop at.
        # Used to control the frequency at which your loop operates
        self.rate = rospy.Rate(10)  # 10hz

    # A helper function to rotate the point if necessary (by default, it returns the point without changes)
    def rotate_gps_point(lat, lon, origin_lat, origin_lon, orientation):
        # If your map is oriented to the true north, you might not need any rotation.
        # Here could be the code to rotate the point based on orientation,
        # but for simplicity, let's skip it as many setups will not need this.
        return lat, lon

    # Function to convert local XY coordinates to GPS (Latitude, Longitude)
    def local_xy_to_lat_lon(x_local, y_local, origin_lat, origin_lon, orientation):
        # Earth’s radius, sphere
        R = 6378137.0
    
        # Offset in radians
        dLat = y_local / R
        dLon = x_local / (R * math.cos(math.pi * origin_lat / 180))
    
        # Output coordinates in decimal degrees
        new_lat = origin_lat + (dLat * 180 / math.pi)
        new_lon = origin_lon + (dLon * 180 / math.pi)
    
        # Apply orientation if your local system is rotated relative to the North
        new_lat, new_lon = rotate_gps_point(new_lat, new_lon, origin_lat, origin_lon, orientation)
    
        return new_lat, new_lon




    def obstacle_detection_callback(self, data):
        width = data.info.width
        height = data.info.height
        resolution = data.info.resolution
        origin = data.info.origin.position

        obstacle_points = []
        for y in range(height):
            for x in range(width):
                if data.data[y * width + x] >= 1:  # Threshold for obstacles
                    real_x = origin.x + (x + 0.5) * resolution
                    real_y = origin.y + (y + 0.5) * resolution
                    obstacle_points.append([real_x, real_y])

        obstacle_present = (obstacle_points != [])
        

        if (obstacle_present):
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
                self.obstacle_radius_publisher.publish(Float32(radius))
            
                # Determine the x and y coordinates of the centroid.
                centroid_x = centroid[0]
                centroid_y = centroid[1]

                # x_local = 
                # y_local = 

                lat, lon = local_xy_to_lat_lon(x_local, y_local, origin_lat, origin_lon, orientation)



           



    def run(self):
        # Keep the program running until shutdown signal is received
        while not rospy.is_shutdown():
            self.rate.sleep()





# Example usage:
origin_lat = 34.0224  # Insert your origin latitude here
origin_lon = -118.2851  # Insert your origin longitude here
centroid_x = 50  # Your computed centroid x-coordinate in meters (from OccupancyGrid processing)
centroid_y = 100  # Your computed centroid y-coordinate in meters
orientation = 0  # Orientation of your local system relative to the North

centroid_lat, centroid_lon = local_xy_to_lat_lon(centroid_x, centroid_y, origin_lat, origin_lon, orientation)
print("Centroid Latitude:", centroid_lat, "Centroid Longitude:", centroid_lon)

if __name__ == '__main__':
    # Create the object of the class ObstacleDetector
    obstacle_detector = ObstacleDetector()
    # Run the object to keep the node alive
    obstacle_detector.run()
