#include "receive_gps.h"

GPS::GPS()
: first_data_flag(false)
{
    ros::NodeHandle nh("~");

    navsatfix_sub = nh.subscribe("/gps_topic", 1, &GPS::navSatFixCallback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gps/pose", 1000);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/odom", 1000);
    gt_traj_pub = nh.advertise<nav_msgs::Path>("/gt_traj_pub", 1000);
    first_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps/first_odom", 1000, true);
}

GPS::~GPS(){
}

void GPS::navSatFixCallback(const sensor_msgs::NavSatFixPtr& fix_msg)
{
    if (fix_msg->header.stamp == ros::Time(0))
    {
        printf("error data\n");
        return;
    }

    if (!first_data_flag)
    {
        string no_use;
        LLtoUTM(fix_msg->latitude, fix_msg->longitude, northing_offset, easting_offset, no_use);
        first_data_flag = true;

        nav_msgs::Odometry first_odom;
        first_odom.header.stamp = fix_msg->header.stamp;
        first_odom.header.frame_id = "earth";
        first_odom.child_frame_id = "world";
        first_odom.pose.pose.position.x = easting_offset;
        first_odom.pose.pose.position.y = northing_offset;
        first_odom.pose.pose.position.z = 0.0;
        first_odom.pose.pose.orientation.x = 0.0;
        first_odom.pose.pose.orientation.y = 0.0;
        first_odom.pose.pose.orientation.z = 0.0;
        first_odom.pose.pose.orientation.w = 1.0;
        first_odom_pub.publish(first_odom);
    }
    string zone;
    double northing, easting;
    LLtoUTM(fix_msg->latitude, fix_msg->longitude, northing, easting, zone);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = fix_msg->header.stamp;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "gps_link";
    odometry.pose.pose.position.x = easting - easting_offset;
    odometry.pose.pose.position.y = northing - northing_offset;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation.x = 0.0;
    odometry.pose.pose.orientation.y = 0.0;
    odometry.pose.pose.orientation.z = 0.0;
    odometry.pose.pose.orientation.w = 1.0;
    odom_pub.publish(odometry);

    static tf::TransformBroadcaster br;
    tf::Transform tf_world_to_map = tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, -PI/2.0), tf::Vector3(0.0, 0.0, 0.0));
    tf::StampedTransform trans_world_to_map = tf::StampedTransform(tf_world_to_map, odometry.header.stamp, "world", "map");
    br.sendTransform(trans_world_to_map);

    tf::Transform tf_world_gps;
    tf::poseMsgToTF(odometry.pose.pose, tf_world_gps);
    tf::Transform tfAfter = tf_world_to_map.inverse() * tf_world_gps;
    
    geometry_msgs::PoseStamped pose_stamped;
    path.header.frame_id = "odom";
    path.header.stamp = fix_msg->header.stamp;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.header.stamp = fix_msg->header.stamp;
    tf::poseTFToMsg(tfAfter, pose_stamped.pose);
    path.poses.push_back(pose_stamped);
    gt_traj_pub.publish(path);    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "receive_gps");

    GPS gps;

    ROS_INFO("\033[1;32m----> receive_gps started\033[0m");

    ros::spin();

    return 0;
}
