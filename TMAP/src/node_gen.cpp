#include <iostream>
#define PCL_NO_PRECOMPILE

#include "include/node_gen.hpp"

ros::Publisher pub_deskew_ptCloud;
ros::Publisher pub_origin_ptCloud;
ros::Publisher pub_gravity_ptCloud;
ros::Publisher pub_LIG_node;

std::string cloud_info_topic;
std::string odom_topic_;
std::string local_frame_;
std::string lidar_frame_;
std::string lidar_align_frame_;
int window_size_;

boost::shared_ptr<RoughPtDeskew> roughdeskew;

std::mutex cloudLock_;
std::mutex odomLock_;

std::deque<lvi_sam::cloud_info> cloudQueue_;
std::deque<nav_msgs::Odometry> odomQueue_;
lvi_sam::cloud_info current_cloud_node_;

void callbackCloudInfo(const lvi_sam::cloud_info::ConstPtr &cloud_info_msg){
    // ROS_INFO_STREAM("LiDAR data in...");

    cloudLock_.lock();
    cloudQueue_.push_back(*cloud_info_msg);
    cloudLock_.unlock();
    // std::cout<< "PtCloud Queue size: "<< cloudQueue_.size() <<std::endl;
    
    // pcl::PointCloud<ouster_ptype::PointOS0> temp_ptCloud = ptHandle::cloudmsg2cloud<ouster_ptype::PointOS0>(*pc_msg);
    // std::cout <<"\033[1;32m" << "cloud stamp: "  << temp_ptCloud.header.stamp << "\033[0m" <<std::endl;

    // std::cout <<"\033[1;32m" << "point MIN stamp: "  << temp_ptCloud.points.front().t << "\033[0m" <<std::endl;
    // std::cout <<"\033[1;32m" << "point MAX stamp: "  << temp_ptCloud.points.back().t << "\033[0m" <<std::endl;
    // std::cout << " " <<std::endl;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr &odom_msg){

    odomLock_.lock();
    odomQueue_.push_back(*odom_msg);
    odomLock_.unlock();

}

void process(){
    std::cout<<"Thread: deskewPtCloud" <<std::endl;
    while(true){
        std::this_thread::sleep_for(std::chrono::duration<double>(0.05));

        // Check data in
        if (cloudQueue_.empty() || odomQueue_.size()<10 ){
            // if (cloudQueue_.empty())  ROS_WARN_STREAM("Wait for LiDAR data");
            // if (odomQueue_.size()<10) ROS_WARN_STREAM("Wait for ODOM  data");
            continue;
        }

        //get ptCloud
        cloudLock_.lock();
        if ( !(roughdeskew->setPtCloud(cloudQueue_.front()))) {
            cloudLock_.unlock();
            continue;
        }
        current_cloud_node_ = cloudQueue_[0];
        cloudQueue_.pop_front();
        cloudLock_.unlock();

        //get Odometry
        odomLock_.lock();
        if (!(roughdeskew->setOdomQueue(odomQueue_))){
            odomLock_.unlock();
            continue;
        } else {
            for (int i=0; i<roughdeskew->nextODOMInitIdx;i++){
                odomQueue_.pop_front();
            }
        }
        odomLock_.unlock();
        
        // Process data
        // ptCloud deskewing
        pcl::PointCloud<pcl::PointXYZI> origin_ptCloud;
        pcl::PointCloud<pcl::PointXYZI> gAlign_ptCloud;
        if (!roughdeskew->ptCloudProcess(origin_ptCloud, gAlign_ptCloud)){
            ROS_ERROR_STREAM("Error in deskewing");
            continue;
        } else {
            tMap_msgs::TrvMapDet_node TrvMap_node;

            TrvMap_node.header.stamp = roughdeskew->getLiDARheader().stamp;
            TrvMap_node.header.frame_id = lidar_frame_;
            
            TrvMap_node.ptCloud_raw = ptHandle::cloud2msg(origin_ptCloud,   lidar_frame_);
            TrvMap_node.ptCloud_gAligned = ptHandle::cloud2msg(gAlign_ptCloud, lidar_align_frame_);

            TrvMap_node.lidar_odom = roughdeskew->getLiDARodom();
            TrvMap_node.lidar_odom2D = roughdeskew->getLiDARodom2D();

            pub_LIG_node.publish(TrvMap_node);

            pub_origin_ptCloud.publish(ptHandle::cloud2msg(origin_ptCloud,   lidar_frame_));
            pub_gravity_ptCloud.publish(ptHandle::cloud2msg(gAlign_ptCloud, lidar_align_frame_));
        }

        static tf::TransformBroadcaster br;
        tf::Transform t_baselink_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0));
        tf::StampedTransform trans_baselink_to_lidar = tf::StampedTransform(t_baselink_to_lidar, roughdeskew->getLiDARheader().stamp, "base_link", lidar_frame_ );
        br.sendTransform(trans_baselink_to_lidar);
    }
}

int main(int argc, char **argv)
{

    // ROS Init
    ros::init(argc, argv, "ptCloud_deskewer");
    
    ROS_INFO_STREAM("PointCloud Deskewing node...");
    ros::NodeHandle nh_;

    nh_.param<std::string>("/cloud_info_topic"  , cloud_info_topic, "/os_cloud_node/points");
    nh_.param<std::string>("/odom_topic" , odom_topic_, "/odometry/imu_incremental");
    nh_.param<std::string>("/nodegen/local_frame", local_frame_, "base_link");
    nh_.param<std::string>("/nodegen/lidar_frame", lidar_frame_, "os_sensor");
    nh_.param<std::string>("nodegen/lidar_align_frame", lidar_align_frame_, "sensor_align");
    
    ROS_INFO_STREAM("PC  Topic   : " + cloud_info_topic);
    ROS_INFO_STREAM("Odom Topic   : " + odom_topic_);

    //Node Publisher
    pub_LIG_node = nh_.advertise<tMap_msgs::TrvMapDet_node>("/lig_node/pre_processing_node",1);

    //PtCloud Publisher
    pub_origin_ptCloud = nh_.advertise<sensor_msgs::PointCloud2>("/origin_ptCloud",1);
    pub_gravity_ptCloud= nh_.advertise<sensor_msgs::PointCloud2>("/GAlign_ptCloud",1);

    //Subscriber
    ros::Subscriber sub_CloudInfo = nh_.subscribe<lvi_sam::cloud_info>(cloud_info_topic, 100, callbackCloudInfo, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_Odom  = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 2000, callbackOdom, ros::TransportHints().tcpNoDelay());

    roughdeskew.reset(new RoughPtDeskew(&nh_));

    // Register signal and signal handler
    signal(SIGINT, signal_handle::signal_callback_handler);

    // Node generator
    std::thread processPtCloud{process};
    ros::spin();

    // ros::MultiThreadedSpinner spinner(2);
    processPtCloud.join();
    return 0;
}


