#include <iostream>
#define PCL_NO_PRECOMPILE

#include <chrono>
#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/eigen.h>
#include <velodyne_pointcloud/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "tMap_msgs/TrvMapDet_node.h"
#include "lvi_sam/cloud_info.h"
#include "../../utils/point_types.hpp"
#include "../../utils/ptcloud_handle.hpp"
#include "../../utils/imu_handle.hpp"
#include "../../utils/signal_handle.hpp"

const int queueLength = 2000;

class RoughPtDeskew {
public:    
    RoughPtDeskew(){};
    RoughPtDeskew(ros::NodeHandle* nh):node_handle_(*nh)
    {   
        node_handle_.param<std::string>("/nodegen/local_frame", local_frame_, "base_link");
        node_handle_.param<std::string>("/nodegen/lidar_frame", lidar_frame_, "os_sensor");
        node_handle_.param<std::string>("nodegen/lidar_align_frame", lidar_align_frame_, "sensor_align");

        node_handle_.param<double>("nodegen/min_range", min_range_, 0.0);
        node_handle_.param<double>("nodegen/max_range", max_range_, 80.0);
        
        current_ptCloud_.reserve(NUM_POINTCLOUD_MAXNUM);
        gravity_align_ptCloud_.reserve(NUM_POINTCLOUD_MAXNUM);

    }
    ~RoughPtDeskew(){
    }

    bool setPtCloud(lvi_sam::cloud_info& cloud_msg);

    bool setOdomQueue(std::deque<nav_msgs::Odometry>& odomQ);

    bool ptCloudProcess(pcl::PointCloud<pcl::PointXYZI> &origin_ptCloud, pcl::PointCloud<pcl::PointXYZI> &gAlign_ptCloud);

    geometry_msgs::Pose getLiDARodom(){
        return scanInit_LiDARpose_;
    }
    geometry_msgs::Pose getLiDARodom2D(){
        return scanInit_LiDARpose_2D_;
    }
    std_msgs::Header getLiDARheader(){
        return cloudHeader_;
    }

    int nextODOMInitIdx = 0;

    pcl::PointCloud<pcl::PointXYZI> current_ptCloud_;
    std::vector<nav_msgs::Odometry> current_OdomQ_;

private: 
    ros::NodeHandle node_handle_;

    pcl::PointCloud<pcl::PointXYZI> gravity_align_ptCloud_;
    std_msgs::Header cloudHeader_;
    std::string local_frame_;
    std::string lidar_frame_;
    std::string lidar_align_frame_;

    double scanInitT_ = 0;
    double scanEndT_  = 0;
    
    bool firstPointFlag_ = true;
    double min_range_;
    double max_range_;

    Eigen::Affine3f Rotation_GAlign_;

    geometry_msgs::Pose scanInit_LiDARpose_;
    geometry_msgs::Pose scanInit_LiDARpose_2D_;

    bool setParams();
    void setGAlignTF();
    void gAlignPtCloud();
};

bool RoughPtDeskew::setPtCloud(lvi_sam::cloud_info& cloud_msg){
    
    current_ptCloud_.clear();

    current_ptCloud_ =  ptHandle::cloudmsg2cloud<pcl::PointXYZI>(cloud_msg.cloud_deskewed);
    // get timestamp
    cloudHeader_ = cloud_msg.header;
    scanInitT_   = cloudHeader_.stamp.toSec();
    scanEndT_    = scanInitT_ + 0.09;
    
    return true;
}

bool RoughPtDeskew::setOdomQueue(std::deque<nav_msgs::Odometry>& odomQ){
    current_OdomQ_.clear();
    int odomCur = 0;
    nextODOMInitIdx = 0;

    if (!(odomQ.back().header.stamp.toSec() >= scanEndT_))
        return false;

    for (int i = 0; i < (int)odomQ.size();i++){
        nav_msgs::Odometry thisOdomMsg = odomQ[i];
        double thisOdomTime = thisOdomMsg.header.stamp.toSec();

        if (thisOdomTime > scanInitT_){
            //set init 
            if (odomCur == 0 && i !=0 ){
                current_OdomQ_.push_back(odomQ[i-1]);
                odomCur++;
            }

            //just before scan start just after scan end
            current_OdomQ_.push_back(thisOdomMsg);
            odomCur++;

            // Up to the first odom, which is given after scan.
            if (thisOdomTime > scanEndT_){
                nextODOMInitIdx = i-1;
                break;
            }

            // End before over the scan time.
            if (i == (int)odomQ.size()-1){
                nextODOMInitIdx = i;
                break;
            }
        } else {
            // ignore the old odom data.
        }
    }

    if (odomCur <= 0){
        return false;
    }

    return true;
}

bool RoughPtDeskew::setParams(){

    if (!(current_OdomQ_.front().header.stamp.toSec() <= scanInitT_) || !(current_OdomQ_.back().header.stamp.toSec()  >= scanEndT_ ) ){
        ROS_ERROR("Wait for enough odom data...");
        return false;
    }
    // std::cout << "Current PtCloud  :  "<< current_ptCloud_.size() << std::endl;
    // std::cout << "Current OdomQueue: " << current_OdomQ_.size() << std::endl;

    firstPointFlag_ = true;
    
    gravity_align_ptCloud_.clear();

    // Get Odometry in the start of scan.
    nav_msgs::Odometry startOdomMsg;
    nav_msgs::Odometry secondOdomMsg;
    for (int idx = 0; idx<(int)current_OdomQ_.size() ; idx++){
        if (current_OdomQ_[idx].header.stamp.toSec() <= scanInitT_){
            startOdomMsg = current_OdomQ_[idx];
            secondOdomMsg = current_OdomQ_[idx+1];
            continue;
        } else{
            break;
        }
    }

    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    
    double roll2, pitch2, yaw2;
    tf::Quaternion orientation2;
    tf::quaternionMsgToTF(secondOdomMsg.pose.pose.orientation, orientation2);
    tf::Matrix3x3(orientation2).getRPY(roll2, pitch2, yaw2);

    // linear interporlation: There is no big difference in the result even if you don't do it because the time doesn't differ much and the speed of vehicle is low.
    double ratio_after  = (scanInitT_ - startOdomMsg.header.stamp.toSec()) /(secondOdomMsg.header.stamp.toSec()-startOdomMsg.header.stamp.toSec());
    double ratio_before = (secondOdomMsg.header.stamp.toSec() - scanInitT_)/(secondOdomMsg.header.stamp.toSec()-startOdomMsg.header.stamp.toSec());

    scanInit_LiDARpose_.position.x = startOdomMsg.pose.pose.position.x*ratio_before + secondOdomMsg.pose.pose.position.x*ratio_after;
    scanInit_LiDARpose_.position.y = startOdomMsg.pose.pose.position.y*ratio_before + secondOdomMsg.pose.pose.position.y*ratio_after;
    scanInit_LiDARpose_.position.z = startOdomMsg.pose.pose.position.z*ratio_before + secondOdomMsg.pose.pose.position.z*ratio_after;

    roll = roll*ratio_before + roll2*ratio_after;
    pitch= pitch*ratio_before+ pitch2*ratio_after;
    yaw  = yaw*ratio_before  + yaw2*ratio_after;

    scanInit_LiDARpose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    scanInit_LiDARpose_2D_.position = scanInit_LiDARpose_.position;
    scanInit_LiDARpose_2D_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    
    return true;
}

void RoughPtDeskew::setGAlignTF(){
    double gravityRoll, gravityPitch, gravityYaw;

    // Use Odom data 
    tf::Quaternion current_LiDARorientation;
    tf::quaternionMsgToTF(scanInit_LiDARpose_.orientation, current_LiDARorientation);
    tf::Matrix3x3(current_LiDARorientation).getRPY(gravityRoll, gravityPitch, gravityYaw);
    std::cout << "Platform roll: "<< gravityRoll*180.0/M_PI << ", Pitch: " << gravityPitch*180.0/M_PI << ", Yaw: "<< gravityYaw*180.0/M_PI << std::endl;

    // Gravity Aligned TF send.
    Rotation_GAlign_ = (pcl::getTransformation(0,0,0,-gravityRoll, -gravityPitch, 0));

    static tf::TransformBroadcaster br;
    tf::Transform t_lidar_to_galign = tf::Transform(tf::createQuaternionFromRPY(-gravityRoll, -gravityPitch,0), tf::Vector3(0,0,0));
    tf::StampedTransform trans_lidar_to_align = tf::StampedTransform(t_lidar_to_galign, cloudHeader_.stamp, lidar_frame_, lidar_align_frame_);
    br.sendTransform(trans_lidar_to_align);
    return;
}

void RoughPtDeskew::gAlignPtCloud(){

    for ( pcl::PointXYZI thisPt : current_ptCloud_.points){
        
        //range filter
        float range = ptHandle::pointDistance(thisPt);
        if (range < min_range_ || range > max_range_){ continue; }

        // Define the relative transformation in poinftcloud.
        Eigen::Affine3f transBt_G   = Rotation_GAlign_.inverse();

        pcl::PointXYZI gAlign_pt;
        gAlign_pt.x = transBt_G(0,0) * thisPt.x + transBt_G(0,1) * thisPt.y + transBt_G(0,2) * thisPt.z + transBt_G(0,3);
        gAlign_pt.y = transBt_G(1,0) * thisPt.x + transBt_G(1,1) * thisPt.y + transBt_G(1,2) * thisPt.z + transBt_G(1,3);
        gAlign_pt.z = transBt_G(2,0) * thisPt.x + transBt_G(2,1) * thisPt.y + transBt_G(2,2) * thisPt.z + transBt_G(2,3);
        gAlign_pt.intensity = thisPt.intensity;

        gravity_align_ptCloud_.push_back(gAlign_pt);
    }
}

bool RoughPtDeskew::ptCloudProcess(pcl::PointCloud<pcl::PointXYZI> &origin_ptCloud, pcl::PointCloud<pcl::PointXYZI> &gAlign_ptCloud){
    // current_ptCloud_
    std::chrono::system_clock::time_point s = std::chrono::system_clock::now();

    if (!setParams()){
        return false;
    }
    
    setGAlignTF();
    gAlignPtCloud();

    origin_ptCloud.clear();
    gAlign_ptCloud.clear();
    pcl::copyPointCloud(current_ptCloud_, origin_ptCloud);
    pcl::copyPointCloud(gravity_align_ptCloud_, gAlign_ptCloud);
    
    std::chrono::duration<double> sec = std::chrono::system_clock::now() - s;
    std::cout<<"\033[34;1m"<<"NodeGen Process: " << sec.count() << " seconds" <<  "\033[32;0m" << std::endl;
    std::cout<<" "<<std::endl;
    return true;
}