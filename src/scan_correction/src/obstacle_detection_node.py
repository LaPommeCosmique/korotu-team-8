#!/usr/bin/env python 
  
import rospy 
from std_msgs.msg import String 
from nav_msgs.msg import OccupancyGrid
import numpy as np
from sklearn.cluster import DBSCAN


# Function that runs when this node is exectuded.  
def main():
        # initialize a node by the name 'obstacle_detection'.
        rospy.init_node('obstacle_detection', anonymous=True)
        # Subscribe to the following topics.
        rospy.Subscriber("/map", OccupancyGrid, obstacle_detection_callback)
        # spin() simply keeps python from exiting until this node is stopped 
        rospy.spin()
    
# Other programmer-defined functions used by main().

# Function 1: callback function for obtaining obstacle coordinates   
def obstacle_detection_callback(data): 
    width = data.info.width
    height = data.info.height
    resolution = data.info.resolution  # size of map squares in meters
    origin = data.info.origin.position  # The origin of the map [m, m, rad]
    
    # Preparing data for clustering
    obstacle_points = []
    for y in range(height):
        for x in range(width):
            if data.data[y * width + x] >= 1:  # Threshold for obstacles
                real_x = origin.x + (x + 0.5) * resolution
                real_y = origin.y + (y + 0.5) * resolution
                obstacle_points.append([real_x, real_y])
    
    # Clustering obstacle points
    clustering = DBSCAN(eps=resolution*1.5, min_samples=3).fit(obstacle_points)
    labels = clustering.labels_

    # Grouping points by clusters to find centroids and radii
    clusters = {}
    for label, point in zip(labels, obstacle_points):
        if label == -1:  # Ignoring noise points
            continue
        if label not in clusters:
            clusters[label] = []
        clusters[label].append(point)
    
    # Calculating centroids and radii
    for label, points in clusters.items():
        points = np.array(points)
        centroid = points.mean(axis=0)
        radius = np.max(np.sqrt(np.sum((points - centroid)**2, axis=1)))
        
        print(f"Obstacle {label}: Centroid = {centroid}, Radius = {radius} meters")
    
    # Here you can do something with the obstacles, e.g., visualize them or plan a path

    #End of function 1 

    
  
    
   
    
   




  

if __name__ == '__main__': 
      
    # you could name this function 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass



        
## Extra code below that may / may not be needed.

#Algo 1  
# print the actual message in its raw format 
# rospy.loginfo("Here's what was subscribed: %s", data.data) 
      
# otherwise simply print a convenient message on the terminal 
# print('Data from /topic_name received')
#End of algo 1

#Algo 2
  # Figure out the vector going from curent position to target destination.
    # A line of code to reflect: Vector = target destination - current position.
    
    # Figure out how to plot the above vector on the map starting from the current position.
    
    # Figure out if any obstacles (point clouds) lie on the vector line on the map.
    # If yes, determine new coordinates and send this data as a message to the /target_coordinate topic. Need to figure out how to determine the new cooordinates.
    # If no, continue on current path indicated by given vector. Will need to figure out the code to implement this. 
#end of algo 2

#Algo 3
# Could modify and use below code for callback function.
#def avoid_obstacles(self, point_cloud,dt):
#    closest_obs = None
#    dist = np.inf
    
#    if len(point_cloud) > 1:
#    	for point in point_cloud: 
#            if dist > distance([self.x, self.y], point):
#                dist = ditsance([self.x, self.y], point)
#                closest_obs = (point, dist)
                
#        if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
#            self.count_down -= dt
#        else:
#            # reset count down
#            self.count_down = 5
            
#            # move forward
#            self.move_forward()
# End of Algo 3
    
# Algo 4
#def move_forward(self):
#    # Function 
# End of Algo 4

#Algo 5
 #def main(): 
      
    
 #   rospy.init_node('obstacle_avoidance_navigation', anonymous=True) 
    
 #   # Subscribe to the following topics. Might need to modify code to ensure that the callback function is executed only when one message from each of the topics for a given instant in time has been sent to the node. 
 #   rospy.Subscriber("/transformed_map", PointCloud2, obstacle_avoidance_navigation_callback)  # PointCloud2 appears to be the data type of Rviz maps.
 #   rospy.Subscriber("/transformed_pose", matrix, obstacle_avoidance_navigation_callback) # Could have the messages of /transformed_pose topic as a matrix. Need to figure out the data type of a numpy matrix in Python.
 #   # For the above matrix, could have a 4x4 transformation matrix describing the position and orientation of the RPLiDAR coordinate system on the drone.
 #   rospy.Subscriber("/target_destination", vector, obstacle_avoidance_navigation_callback) #Could have the messages of /target_destination topic as a vector. Need to figure out the data type of a numpy vector in Python.
    
 #   #For subscriber lines below, may/may not need callback function.
 #   rospy.Subscriber("/target_velocity", vector, callback)
 #   rospy.Subscriber("/target_coordinate", vector, callback)
    
    
    
    
    
 #   # spin() simply keeps python from 
 #   # exiting until this node is stopped 
 #   rospy.spin() 
#End of Algo 5

# Algo 6
# DO NOT skip the next commented line
#End of Algo 6

#Algo 7
    ## Convert OccupancyGrid data to 2D list for easier processing
    #grid = []
    #for i in range(height):
    #    row = data.data[i*width:(i+1)*width]
    #    grid.append(row)
#End of algo 7

# Algo 8
    ## Find and print obstacle locations
    #obstacles = []
    #for y, row in enumerate(grid):
    #    for x, cell in enumerate(row):
    #        if cell >= 1:  # Assuming cells >= 1 represent obstacles
    #            # Convert grid coordinates to real-world coordinates
    #            real_x = origin.x + (x + 0.5) * resolution
    #            real_y = origin.y + (y + 0.5) * resolution
    #            obstacles.append((real_x, real_y))
    #            print("Obstacle at ({}, {})".format(real_x, real_y))
#End of Algo 8

 
