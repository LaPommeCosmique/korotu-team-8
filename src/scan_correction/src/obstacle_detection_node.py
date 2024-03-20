# DO NOT skip the next commented line 
#!/usr/bin/env python 
  
import rospy 
from std_msgs.msg import String 
import numpy as np

# Function that runs when this node is exectuded.  
 def main(): 
      
    # initialize a node by the name 'obstacle_avoidance_navigation'.
    rospy.init_node('obstacle_avoidance_navigation', anonymous=True) 
    
    # Subscribe to the following topics. Might need to modify code to ensure that the callback function is executed only when one message from each of the topics for a given instant in time has been sent to the node. 
    rospy.Subscriber("/transformed_map", PointCloud2, obstacle_avoidance_navigation_callback)  # PointCloud2 appears to be the data type of Rviz maps.
    rospy.Subscriber("/transformed_pose", matrix, obstacle_avoidance_navigation_callback) # Could have the messages of /transformed_pose topic as a matrix. Need to figure out the data type of a numpy matrix in Python.
    # For the above matrix, could have a 4x4 transformation matrix describing the position and orientation of the RPLiDAR coordinate system on the drone.
    rospy.Subscriber("/target_destination", vector, obstacle_avoidance_navigation_callback) #Could have the messages of /target_destination topic as a vector. Need to figure out the data type of a numpy vector in Python.
    
    #For subscriber lines below, may/may not need callback function.
    rospy.Subscriber("/target_velocity", vector, callback)
    rospy.Subscriber("/target_coordinate", vector, callback)
    
    
    
    
    
    # spin() simply keeps python from 
    # exiting until this node is stopped 
    rospy.spin() 
   
def obstacle_avoidance_navigation_callback(data): 
      
     
    
    # Figure out the vector going from curent position to target destination.
    # A line of code to reflect: Vector = target destination - current position.
    
    # Figure out how to plot the above vector on the map starting from the current position.
    
    # Figure out if any obstacles (point clouds) lie on the vector line on the map.
    # If yes, determine new coordinates and send this data as a message to the /target_coordinate topic. Need to figure out how to determine the new cooordinates.
    # If no, continue on current path indicated by given vector. Will need to figure out the code to implement this. 
    
   
    
   


# Could modify and use below code for callback function.
def avoid_obstacles(self, point_cloud,dt):
    closest_obs = None
    dist = np.inf
    
    if len(point_cloud) > 1:
    	for point in point_cloud: 
            if dist > distance([self.x, self.y], point):
                dist = ditsance([self.x, self.y], point)
                closest_obs = (point, dist)
                
        if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
            self.count_down -= dt
        else:
            # reset count down
            self.count_down = 5
            
            # move forward
            self.move_forward()
            
def move_forward(self):
    # Function 

  

if __name__ == '__main__': 
      
    # you could name this function 
    try: 
        main() 
    except rospy.ROSInterruptException: 
        pass
        
## Extra code below that may / may not be needed.

#Code 1  
# print the actual message in its raw format 
# rospy.loginfo("Here's what was subscribed: %s", data.data) 
      
# otherwise simply print a convenient message on the terminal 
# print('Data from /topic_name received')
#End of code 1



 

