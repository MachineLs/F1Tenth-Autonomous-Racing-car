#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
#include <vector>
#include <cmath>
#include <iostream>

visualization_msgs::Marker points, line_strip, line_strip2;
std::vector<float> intensities, ranges;
float distanceThreshold = 3.0;
//Magic number comes from (PI/2)*2/ANGLE_INCREMENT = 360 degrees
//Where PI/2 is the angle range, 180 degrees is the FOV
static double ANGLE_INCREMENT = 0.00872664619237;
static int TOLERANCE = 3; //The tolerance for RANSAC
static int SAMPLES = 361;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	ranges = msg->ranges;
	intensities = msg->intensities;
}

geometry_msgs::Point translatePoint(int i) {
	geometry_msgs::Point p;
	double radian = (i * ANGLE_INCREMENT) - (M_PI / 2);
	double radius = ranges[i];

	p.x = radius * cos(radian);
	p.y = radius * sin(radian);
	p.z = 0;
	return p;

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "points_and_lines");

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise < visualization_msgs::Marker
			> ("ransac_vis", 10);

	ros::NodeHandle n2;
	ros::Subscriber sub = n2.subscribe("base_scan", 1000, laserScanCallback);

	ros::Rate r(30);

	while (ros::ok()) {

		points.points.clear();
		line_strip.points.clear();
		line_strip2.points.clear();
		points.header.frame_id = line_strip.header.frame_id =
				line_strip2.header.frame_id = "/base_laser_link";
		points.header.stamp = line_strip.header.stamp =
				line_strip2.header.stamp = ros::Time::now();
		points.ns = line_strip.ns = line_strip2.ns = "points_and_lines";
		points.action = line_strip.action = line_strip2.action =
				visualization_msgs::Marker::ADD;
		points.pose.orientation.w = line_strip.pose.orientation.w =
				line_strip2.pose.orientation.w = 1.0;

		points.id = 0;
		line_strip.id = 1;
		line_strip2.id = 2;

		points.type = visualization_msgs::Marker::POINTS;
		line_strip.type = visualization_msgs::Marker::LINE_STRIP;
		line_strip2.type = visualization_msgs::Marker::LINE_STRIP;

		//Display setup

		//All valid points (GREEN)
		points.scale.x = 0.05;
		points.scale.y = 0.05;
		points.color.g = 1.0;
		points.color.a = 0.4;

		//RANSAC line (RED)
		line_strip2.scale.x = 0.05;
		line_strip2.scale.y = 0.05;
		line_strip2.color.r = 1.0;
		line_strip2.color.a = 1.0;

		//What the bot sees (BLUE)
		line_strip.color.b = 1.0;
		line_strip.color.a = 0.25;
		line_strip.scale.x = 0.1;
		line_strip.scale.y = 0.1;

		int maxIndexStart = 0;
		int maxIndexEnd = 0;
		int maxSize = TOLERANCE;

		if (intensities.size() > SAMPLES - 1) { //initializer check

			int indexStart = 0;
			int indexEnd = 0;
			int iTolerance = 0;
			bool firstValidPoint = false;

			for (int i = 0; i < SAMPLES; ++i) {
				if (intensities[i] == 1) { //Verify that the point is valid

					//Reset the for the next line-fitting
					if (!firstValidPoint) {
						indexStart = i;
						firstValidPoint = true;
					}
					//Point translation
					geometry_msgs::Point p = translatePoint(i);
					points.points.push_back(p);
					line_strip.points.push_back(p);
				} else {
					++iTolerance;
					if (iTolerance == TOLERANCE) {
						iTolerance = 0;
						int size = i - indexStart - TOLERANCE;
						if (firstValidPoint && size > maxSize) { //Used to determine line with most inliers
							maxIndexStart = indexStart;
							maxIndexEnd = indexStart + size;
							maxSize = size;
						}
						firstValidPoint = false;
					}
				}
			}

			//The RANSAC line
			if (maxSize > TOLERANCE + 1) {
				line_strip2.points.push_back(translatePoint(maxIndexEnd));
				line_strip2.points.push_back(translatePoint(maxIndexStart));
			}

			marker_pub.publish(points);
			marker_pub.publish(line_strip);
			marker_pub.publish(line_strip2);
		}

		r.sleep();
		ros::spinOnce();
	}
}

