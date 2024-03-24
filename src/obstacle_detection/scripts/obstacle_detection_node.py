#!/usr/bin/env python

# Import the necessary ROS libraries
import rospy
import message_filters
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Bool
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point32 # Check this based on the topics other than the /map topic.
from sklearn.cluster import DBSCAN

# Note: Might need to change some topic names in the code.

class ObstacleDetector:

    def __init__(self):
        # Initializes the ROS node
        rospy.init_node('obstacle_detector', anonymous=True)

        #Subscriber aspects below.

        # Create message filters for each subscriber topic
        map_sub = message_filters.Subscriber('/map', OccupancyGrid)
        pose_sub = message_filters.Subscriber('/slam_out_pose', PoseStamped) 
        coordinates_sub = message_filters.Subscriber('/coordinates', PointStamped) # Check topic name and message type
        destination_sub = message_filters.Subscriber('/destination', PointStamped) # Check topic name and message type

        # Create an ApproximateTimeSynchronizer object to initiate the call back function only when one message has arrived at each of the above topics
        # with a time difference of maximum 0.1 s between any two of these messages.
        self.ats = message_filters.ApproximateTimeSynchronizer([map_sub, pose_sub, coordinates_sub, destination_sub], 10, 0.1, allow_headerless=True)
        self.ats.registerCallback(self.obstacle_detection_callback)
        
        # Publisher aspects below.

        # Setting up the publishers, each of which publishes to its respective topic. 
        # Respective topic is mentioned as first input in the () portion of each line.
        self.obstacle_radius_publisher = rospy.Publisher('/obstacle_radius', Float32, queue_size=10) 
        self.obstacle_coordinate_publisher = rospy.Publisher('/obstacle_coordinates', Point32, queue_size=10)
        self.obstacle_present_publisher = rospy.Publisher('/obstacle_present', Bool, queue_size=10)

        
        # Setting the rate to sleep the loop at.
        # Used to control the frequency at which your loop operates
        self.rate = rospy.Rate(10)  # 10hz

    
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
      
        return new_lat, new_lon


    def obstacle_detection_callback(self, map_msg, pose_msg, coordinates_msg, destination_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        # Code to figure out destination_msg.point.x and the same.y.
        # This code will take (lat,long cooords.) of next waypoint and convert them to 

        #lat_dest = destination_msg[0] 
        #long_dest = destination_msg[1]

        destination_x = 8 # Random value, need to generalize
        detsination_y = 9 # Random value, need to generalize

        # Current and target positions
        current_x = (pose_msg.pose.position.x - origin.x) / resolution
        current_y = (pose_msg.pose.position.y - origin.y) / resolution
        target_x = (destination_x - origin.x) / resolution # Check destination_x piece.
        target_y = (destination_y - origin.y) / resolution # Check destination_y piece.

        # Initialize the list to store obstacle points
        obstacle_points = []

        # Bresenham's line algorithm (Python adaptation)
        dx = abs(target_x - current_x)
        dy = -abs(target_y - current_y)
        sx = 1 if current_x < target_x else -1
        sy = 1 if current_y < target_y else -1
        err = dx + dy
        while True:
            # Check if current grid cell is occupied
            grid_x, grid_y = int(current_x), int(current_y)
            if 0 <= grid_x < width and 0 <= grid_y < height:  # Check bounds
                index = grid_y * width + grid_x
                if map_msg.data[index] >= 1:  # Occupied cell
                    real_x = origin.x + (grid_x + 0.5) * resolution
                    real_y = origin.y + (grid_y + 0.5) * resolution
                    obstacle_points.append([real_x, real_y])
            if current_x == target_x and current_y == target_y:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                current_x += sx
            if e2 <= dx:
                err += dx
                current_y += sy

        # Now, obstacle_points contains points only occupied cells along the line from current position to target position.
        
        # Determine whether or not an obstacle is present in the current path.
        obstacle_present = (len(obstacle_points) != 0)
        # Publishing the true/false of obstacle presence above to the /obstacle_present topic
        self.obstacle_present_publisher.publish(obstacle_present)

        # 
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

                #x_local = 
                #y_local = 

                lat, lon = local_xy_to_lat_lon(x_local, y_local, origin_lat, origin_lon) # Check this!
                
                coordinates = Point32()
                coordinates.x = lat  # Assuming lat holds your latitude or similar coordinate
                coordinates.y = lon  # Assuming lon holds your longitude or similar coordinate
                coordinates.z = 0.0  # Z can be set to 0 if not used

                # Publishing the coordinates value to the /obstacle_coordinates topic
                self.obstacle_coordinate_publisher.publish(coordinates)

                #  
                
    




           



    def run(self):
        # Keep the program running until shutdown signal is received
        while not rospy.is_shutdown():
            self.rate.sleep()

def main(): 
    # create a subscriber instance 
    sub = basic_subscriber() 
      
    # follow it up with a no-brainer sequence check 
    print('Currently in the main function...') 
      
    # initializing the subscriber node 
    rospy.init_node('listener', anonymous=True) 
    rospy.spin() 



# Example usage: [for computing latitude and longitude of centroid]
#origin_lat = 34.0224  # Insert your origin latitude here
#origin_lon = -118.2851  # Insert your origin longitude here
#centroid_x = 50  # Your computed centroid x-coordinate in meters (from OccupancyGrid processing)
#centroid_y = 100  # Your computed centroid y-coordinate in meters
#orientation = 0  # Orientation of your local system relative to the North

#centroid_lat, centroid_lon = local_xy_to_lat_lon(centroid_x, centroid_y, origin_lat, origin_lon, orientation)
#print("Centroid Latitude:", centroid_lat, "Centroid Longitude:", centroid_lon)

if __name__ == '__main__': 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass

#if __name__ == '__main__':
#    # Create the object of the class ObstacleDetector
#    obstacle_detector = ObstacleDetector()
#    # Run the object to keep the node alive
#    obstacle_detector.run()
