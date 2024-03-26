#!/usr/bin/env python3

# Import the necessary ROS libraries
import rospy
import message_filters
import math
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, Bool
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped, Point32 # Check this based on the topics other than the /map topic.
from sklearn.cluster import DBSCAN


def obstacle_detection_callback(self, map_msg, pose_msg, coordinates_msg, destination_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin = map_msg.info.origin.position

    destination_lat = destination_msg.x 
    destination_long = destination_msg.y 

    # Determine the lat, long coordinates of the sensor.
    lat_sensor = coordinates_msg.x 
    long_sensor = coordinates_msg.y

    # Determine the angle of yaw from the orientation field of the pose_msg.

    quaternion = (
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    )

     # Convert the quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    orientation = yaw

    destination_x_local, destination_y_local = lat_lon_to_local_xy(destination_lat, destination_long, lat_sensor, long_sensor, orientation)

    # Current and target positions
    current_x = (pose_msg.pose.position.x - origin.x) / resolution
    current_y = (pose_msg.pose.position.y - origin.y) / resolution
    target_x = destination_x_local / resolution + current_x 
    target_y = destination_y_local / resolution + current_y 

    # Initialize the list to store obstacle points
    obstacle_points = []

    # Bresenham's line algorithm (Python adaptation). 
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
                break
        if abs(target_x - current_x)*resolution <= 0.0001 and abs(target_y - current_y)*resolution <= 0.0001:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            current_x += sx
        if e2 <= dx:
            err += dx
            current_y += sy

    # Now, obstacle_points contains points only the first occupied cell along the line from current position to target position
    # if an occupied cell is detected along that line. Otherwise, obstacle_points remains null.
        
    # Determine whether or not an obstacle is present in the current path.
    obstacle_present = (len(obstacle_points) != 0)
    # Publishing the true/false of obstacle presence above to the /obstacle_present topic
    self.obstacle_present_publisher.publish(obstacle_present)

    if (obstacle_present):
        # Perform DBSCAN clustering on the obstacle points
        clustering = DBSCAN(eps=resolution*1.5, min_samples=3).fit(obstacle_points)
        labels = clustering.labels_

        # This assumes that you are interested in the first cluster found
        # Change '0' to another number if you are interested in another cluster
        first_cluster_label = labels[0] 

        # Extract points belonging to the first cluster
        first_cluster_points = [point for point, label in zip(obstacle_points, labels) if label == first_cluster_label]
        
        # Compute the centroid and radius of the first cluster
        if first_cluster_points:
            first_cluster_array = np.array(first_cluster_points)
            centroid = first_cluster_array.mean(axis=0)
            radius = np.max(np.sqrt(np.sum((first_cluster_array - centroid) ** 2, axis=1)))
            
            rospy.loginfo(f"First obstacle encountered: Centroid = {centroid}, Radius = {radius} meters")
        
        # Compute the centroid coordinates
        centroid_coords = Point32()
        centroid_coords.x = centroid[0]
        centroid_coords.y = centroid[1]
        centroid_coords.z = 0.0  # Assuming a 2D plane for the occupancy grid
            
        # Determine x_local and y_local to convert to latitude and longitude coordinates.
        x_local = centroid_coords.x - pose_msg.pose.position.x
        y_local = centroid_coords.y - pose_msg.pose.position.y

        # Determine lat, long from x_local and y_local.
        lat, lon = local_xy_to_lat_lon(x_local, y_local, lat_sensor, long_sensor, orientation) 
        
        # Create and set up variable to store obstacle coordinates and radius.
        coords_rad = Point32()
        coords_rad.x = lat  
        coords_rad.y = lon  
        coords_rad.z = radius  

        # Publishing the coordinates and radius values to the /obstacle_coordinates topic
        self.obstacle_coordinates_radius_publisher.publish(coords_rad)



def __init__(self):
    # Initializes the ROS node
    rospy.init_node('obstacle_detector', anonymous=True)

    #Subscriber aspects below.

    # Create message filters for each subscriber topic
    map_sub = message_filters.Subscriber('/map', OccupancyGrid)
    pose_sub = message_filters.Subscriber('/slam_out_pose', PoseStamped) 
    coordinates_sub = message_filters.Subscriber('/coordinates', PointStamped) # Check message type
    destination_sub = message_filters.Subscriber('/destination', PointStamped) # Check message type

    # Create an ApproximateTimeSynchronizer object to initiate the call back function only when one message has arrived at each of the above topics
    # with a time difference of maximum 0.1 s between any two of these messages.
    self.ats = message_filters.ApproximateTimeSynchronizer([map_sub, pose_sub, coordinates_sub, destination_sub], 10, 0.1, allow_headerless=True)
    self.ats.registerCallback(self.obstacle_detection_callback)
        
    # Publisher aspects below.

    # Setting up the publishers, each of which publishes to its respective topic. 
    # Respective topic is mentioned as first input in the () portion of each line.
    self.obstacle_coordinates_radius_publisher = rospy.Publisher('/obstacle_coordinates_radius', Point32, queue_size=10) 
    self.obstacle_present_publisher = rospy.Publisher('/obstacle_present', Bool, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 

def rotate_gps_point(lat, lon, origin_lat, origin_lon, orientation):
    # Earth’s radius, sphere
    R = 6378137.0  # Radius of the Earth in meters

    # Convert origin and point to radians
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Convert lat/lon to Cartesian coordinates
    x = (lon_rad - origin_lon_rad) * R * math.cos(origin_lat_rad)
    y = (lat_rad - origin_lat_rad) * R

    # Rotate the point by the yaw angle
    cos_yaw = math.cos(orientation)
    sin_yaw = math.sin(orientation)
    x_rotated = cos_yaw * x - sin_yaw * y
    y_rotated = sin_yaw * x + cos_yaw * y

    # Convert the rotated Cartesian coordinates back to lat/lon
    new_lon = origin_lon_rad + x_rotated / (R * math.cos(origin_lat_rad))
    new_lat = origin_lat_rad + y_rotated / R

    # Convert back from radians to degrees
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)

    return new_lat, new_lon
    
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

def lat_lon_to_local_xy(lat, lon, origin_lat, origin_lon, orientation):
    # orientation (float): Orientation of the local coordinate system in radians.
    
    # Earth’s radius, sphere
    R = 6378137.0

    # Convert latitude and longitude to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    # Calculate differences
    dLat = lat_rad - origin_lat_rad
    dLon = lon_rad - origin_lon_rad

    # Calculate local x and y
    y_local = dLat * R
    x_local = dLon * R * math.cos(origin_lat_rad)
    
    # If there is an orientation, rotate the point
    if orientation != 0:
        cos_o = math.cos(-orientation)  # Negate because we're transforming from global to local
        sin_o = math.sin(-orientation)
        x_rotated = x_local * cos_o - y_local * sin_o
        y_rotated = x_local * sin_o + y_local * cos_o
        x_local, y_local = x_rotated, y_rotated

    return x_local, y_local


if __name__ == '__main__': 
    __init__(self)
    


