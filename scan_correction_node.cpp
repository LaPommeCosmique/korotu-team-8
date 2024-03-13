/**
 * A program that takes in IMU and scan data and publishes the scan data after correcting it to a fixed orientation
 * @author Alex Ge, UBC MECH 457 Capstone Team 8
 *         Template from Addison Sears-Collins (https://automaticaddison.com/)
 * @version 1.0
 */

// Include the header file that has declarations for the standard ROS classes
#include "ros/ros.h"

// Header file for messages
// Each type of message you use in a program will have a C++ header file you
// need to include. The syntax is #include <package_name/type_name.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// Header file for other objects
#include <boost/array.hpp>
#include <vector>
#include <cmath>

// ROS subscriber and publisher objects.
static ros::Subscriber imuSub;
static ros::Subscriber scanSub;
static ros::Publisher correctedScanPub;

// Current z-axis orientation (yaw) according to IMU data
static float yaw = 0;

/**
 * This function is known as a callback function. A program that subscribes to
 * to a topic (i.e. subscriber node) doesn't know when messages will be
 * arriving. If you have code that needs to process an incoming message
 * (e.g. most often sensor data in robotics), you place that code inside a
 * callback function. A callback function is a function that is called once
 * for each message that arrives.
 * The syntax for a subscriber callback function is:
 *   void function_name(const package_name::type_name &msg) {
 *     [insert your code here]
 *   }
 * The function below is executed each time a new imu message arrives.
 *   Topic Name: /mavros/imu/data
 *   Message Type: sensor_msgs/Imu
 */
void imuMessageCallback(const sensor_msgs::Imu::ConstPtr& imuMsg) {

	// save all the data in the imu message to local variables
	std_msgs::Header header = imuMsg->header;
	geometry_msgs::Quaternion orientation = imuMsg->orientation;
	boost::array<double, 9> orientation_covariance = imuMsg->orientation_covariance;
	geometry_msgs::Vector3 angular_velocity = imuMsg->angular_velocity;
	boost::array<double, 9> angular_velocity_covariance = imuMsg->angular_velocity_covariance;
	geometry_msgs::Vector3 linear_acceleration = imuMsg->linear_acceleration;
	boost::array<double, 9> linear_acceleration_covariance = imuMsg->linear_acceleration_covariance;

	// calculate z-axis rotation (yaw) from orientation (quaternion
	double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
	double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
	yaw = std::atan2(siny_cosp, cosy_cosp);
}
/*
 * The function below is executed each time a new scan message arrives.
 *   Topic Name: /scan
 *   Message Type: sensor_msgs/LaserScan
 */
void scanMessageCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg) {

	// Save all the data in the scan message to local variables  
	std_msgs::Header header = scanMsg->header;
	float angle_min = scanMsg->angle_min;
	float angle_max = scanMsg->angle_max;
	float angle_increment = scanMsg->angle_increment;
	float time_increment = scanMsg->time_increment;
	float scan_time = scanMsg->scan_time;
	float range_min = scanMsg->range_min;
	float range_max = scanMsg->range_max;
	std::vector<float> ranges = scanMsg->ranges;
	std::vector<float> intensities = scanMsg->intensities;

	// Offset angle_min and angle_max with current yaw
	angle_min += yaw;
	angle_max += yaw;

	// Create scan message with adjusted orientation
	sensor_msgs::LaserScan correctedScanMsg;

	// Add data to new scan message
	correctedScanMsg.header = header;
	correctedScanMsg.angle_min = angle_min;
	correctedScanMsg.angle_max = angle_max;
	correctedScanMsg.angle_increment = angle_increment;
	correctedScanMsg.time_increment = time_increment;
	correctedScanMsg.scan_time = scan_time;
	correctedScanMsg.range_min = range_min;
	correctedScanMsg.range_max = range_max;
	correctedScanMsg.ranges = ranges;
	correctedScanMsg.intensities = intensities;

	// Publish new scan message to corrected scan message topic
	correctedScanPub.publish(correctedScanMsg);

}

/**
 * Main method
 */
int main(int argc, char** argv) {

	// Initialize the node and set the name.
	// The ros::init() function needs to see argc and argv.
	// The third argument is the name of the node.
	ros::init(argc, argv, "scan_correction_node");

	// Create the main access point for the node
	// This piece of code enables the node to communicate with the ROS system.
	ros::NodeHandle n;

	// This code below is how we subscribe to a topic.
	// The syntax is:
	//   ros::Subscriber sub = node_handle.subscribe(
	//     topic_name_without_leading_slash,
	//     queue_size, callback_function);
	// I like to use a large number like 1000 for the queue size.
	// When new messages arrive on a topic, they line up in a queue until the
	// callback function is executed by ROS. If the queue is full when a new 
	// message arrives, the oldest unprocessed messages are removed to create
	// space for the new message.
	// To prevent overflow of the queue, you'll need to frequently run the 
	// ros::spinOnce or ros::spin function to make sure the callback 
	// function is executed.

	// subscribe to imu data (from flight controller) and scan data (from RPLidar)
	imuSub = n.subscribe("mavros/imu/data", 1000, imuMessageCallback);
	scanSub = n.subscribe("scan", 1000, scanMessageCallback);

	// Create a Publisher object.
	// We use the advertise() function to tell the ROS Master the type of
	// messages (e.g. strings) we want to publish to the /message topic. 
	// The second parameter to advertise() is the size of the message queue
	// used for publishing messages.  If our node is publishing messages
	// at a rate that is too fast for the ROS system to handle, 
	// 1000 messages will be kept in the queue before deleting the oldest unsent 
	// messages.  
	// The syntax is ros::Publisher pub = node_handle.advertise<message_type>(
	//   topic_name, queue_size);
	// If you'd like to publish messages on more than one topic, you need to 
	// create separate ros:Publisher object for each topic you publish too.

	// Create a new publisher to publish to the "corrected_scan" topic
	correctedScanPub = n.advertise<sensor_msgs::LaserScan>("corrected_scan", 1000);


	// This code enters a loop, continually processing messages that arrive
	// in the queue. If you don't have this code, messages that are waiting in
	// the queue will never be processed.
	// Note, if you use ros::spinOnce() instead of ros::spin(), unprocessed 
	// messages will be processed and then the code below this line will 
	// start executing. 
	// With ros::spin(), you want ROS to wait for messages and
	// process those messages until you either hit Ctrl + C on your keyboard
	// or close down this node.
	// Here we use ros::spin() because the node's sole job is to echo (to the 
	// (terminal window) the message that it receives. However, if you have a 
	// node that needs to do something other than just printing
	// messages to a terminal screen, use 
	// ros::spinOnce(), since that gives you more control when you want the node
	// to process incoming messages.
	ros::spin();

	// Code was executed successfully
	return 0;
}
