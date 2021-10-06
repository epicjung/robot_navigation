#ifndef RECEIVE_GPS_H
#define RECEIVE_GPS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include "conversions.h"
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace gps_common;

#define PI 3.141592

class GPS
{
public:
	GPS();
	~GPS();

private:
	ros::NodeHandle nh;
	ros::Subscriber navsatfix_sub;
	ros::Publisher pose_pub;
	ros::Publisher gt_traj_pub;
	ros::Publisher odom_pub;
	ros::Publisher first_odom_pub;
	bool first_data_flag;
    double northing_offset, easting_offset;
	nav_msgs::Path path;


	void navSatFixCallback(const sensor_msgs::NavSatFixPtr& fix_msg);

};

#endif
