#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <tf/tf.h>

enum STATE {
	GOAL_SEEK, WALL_FOLLOW
};

std::vector<float> intensities, ranges;
float distanceThreshold = 3.0;
static int SAMPLES = 361;
static int MIDPOINT_INDEX = SAMPLES / 2;
static float DISTANCE_THRESHOLD_LEFT = 1.0;
static float DISTANCE_THRESHOLD_MID = 0.5;
static int LEFT_FOLLOW_INDEX = SAMPLES - 1;
float midDistance = 0;
float leftDistance = 0;
ros::Publisher velocity_publisher;
bool turnRight = true;
float oldX = 0;

STATE currentState = GOAL_SEEK;

struct POSE {
	float x, y, z, theta;
} myPose, goalPose;

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	myPose.x = msg->pose.pose.position.x;
	myPose.y = msg->pose.pose.position.y;
	myPose.z = msg->pose.pose.position.z;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	myPose.theta = tf::getYaw(q);
}

float getDistance(float cX, float cY, float dX, float dY) {
	return sqrt(pow((dX - cX), 2) + pow((dY - cY), 2));
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	leftDistance = msg->ranges[LEFT_FOLLOW_INDEX];
	midDistance = msg->ranges[MIDPOINT_INDEX];
	intensities = msg->intensities;
	geometry_msgs::Twist tMsg;
	if (currentState == GOAL_SEEK) {
		if (midDistance < DISTANCE_THRESHOLD_MID) {
			currentState = WALL_FOLLOW;
			oldX = myPose.x;
			std::cout << "Switching to wall follow mode" << std::endl;
		} else {
			float angleR = atan2(goalPose.y - myPose.y, goalPose.x - myPose.x);
			if (angleR < 0)
				angleR += 2 * M_PI;
			if (myPose.theta < 0)
				myPose.theta += 2 * M_PI;

			tMsg.angular.z = angleR - myPose.theta;
			tMsg.linear.x = 2;
		}
	} else if (currentState == WALL_FOLLOW) {
		//Must turn right to hug the wall left
		if (midDistance <= DISTANCE_THRESHOLD_MID) {
			tMsg.angular.z = -2;
		} else if (leftDistance > DISTANCE_THRESHOLD_LEFT) { //stay hugging the wall left
			tMsg.angular.z = 1;
		} else {
			if(myPose.x < oldX) {
				currentState = GOAL_SEEK;
				std::cout << "Switching to goal seek mode" << std::endl;
			} else {
				tMsg.linear.x = 2;
			}
		}
	}
	velocity_publisher.publish(tMsg);
}

int main(int argc, char** argv) {

	goalPose.x = 4.5;
	goalPose.y = 9.0;

	ros::init(argc, argv, "points_and_lines");

	ros::NodeHandle n;
	velocity_publisher = n.advertise < geometry_msgs::Twist > ("cmd_vel", 1000);

	ros::NodeHandle n2;
	ros::Subscriber sub = n2.subscribe("base_scan", 1000, laserScanCallback);

	ros::NodeHandle n3;
	ros::Subscriber pose_subscriber = n3.subscribe("base_pose_ground_truth",
			1000, poseCallback);

	ros::Rate r(30);

	while (ros::ok()
			&& (getDistance(myPose.x, myPose.y, goalPose.x, goalPose.y) > 0.8)) {
		r.sleep();
		ros::spinOnce();
	}
	std::cout << "Arrived at destination!" <<std::endl;
}

