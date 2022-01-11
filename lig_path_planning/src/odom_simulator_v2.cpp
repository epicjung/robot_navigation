// 
/*
 *  odom_simulator.cpp
 *  This is to simulate the part Euigon will output
 *  Input: Satellite
 *  Output: nav_msgs::Odometry
 *      frame_id: odom
 *      child_frame_id: base_footprint
 * */

#include <algorithm>
#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Pose.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
// #include <nav_msgs/GetPlan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <gps_common/conversions.h>

class Odom_sim {
    private:
        ros::NodeHandle nh_;    
        ros::Subscriber sub_;
        
        tf2_ros::TransformBroadcaster tf2_Map2CostmapCenter;
        
        // CostmapCenter    : the UTM position of the global costmap center point
        // odom             : the UTM position of the robot at start (when slam starts in SLAM mode or when GPS is first received in GPS mode)
        geometry_msgs::Pose utm_CostmapCenter, utm_odom;

    public:
        bool once_ = true;
        /**
         *  @brief This function publishes the transform from pp_map to map
         */    
        void publish_tf(){
            tf2::Transform T_odom = tf2::Transform( 
                    tf2::Quaternion(utm_odom.orientation.x, utm_odom.orientation.y, utm_odom.orientation.z, utm_odom.orientation.w),
                    tf2::Vector3(utm_odom.position.x, utm_odom.position.y, utm_odom.position.z));

            tf2::Transform T_costmap = tf2::Transform( 
                    tf2::Quaternion(utm_CostmapCenter.orientation.x, utm_CostmapCenter.orientation.y, utm_CostmapCenter.orientation.z, utm_CostmapCenter.orientation.w),
                    tf2::Vector3(utm_CostmapCenter.position.x, utm_CostmapCenter.position.y, utm_CostmapCenter.position.z));

            tf2::Transform T_odom_costmap = T_odom.inverse()* T_costmap;
            geometry_msgs::TransformStamped ts;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id= "map"; 
            ts.child_frame_id = "costmapcenter";
            ts.transform.translation.x = T_odom_costmap.getOrigin().x();
            ts.transform.translation.y = T_odom_costmap.getOrigin().y();
            ts.transform.translation.z = T_odom_costmap.getOrigin().z();
            ts.transform.rotation.x = T_odom_costmap.getRotation().x();
            ts.transform.rotation.y = T_odom_costmap.getRotation().y();
            ts.transform.rotation.z = T_odom_costmap.getRotation().z();
            ts.transform.rotation.w = T_odom_costmap.getRotation().w();
            tf2_Map2CostmapCenter.sendTransform(ts);
        }

        /**
         * @brief  Constructor
         */    
        Odom_sim(ros::NodeHandle & nh): nh_(nh){
            
            std::cout<<"new Odom Sim object created with SLAM mode"<<std::endl;
            this->sub_ = this->nh_.subscribe("/receive_gps/first_odom", 1, &Odom_sim::transformer, this);

            // The Global GPS position is the Global costmap's center GPS position
            double utm_nort, utm_east;
            gps_common::UTM(36.3683644417,127.362330045,&utm_nort,&utm_east);
            utm_CostmapCenter.position.x = utm_nort;
            utm_CostmapCenter.position.y = utm_east;
            utm_CostmapCenter.position.z = 0;
            utm_CostmapCenter.orientation.x = 0;
            utm_CostmapCenter.orientation.y = 0;
            utm_CostmapCenter.orientation.z = 0;
            utm_CostmapCenter.orientation.w = 1;
            ROS_WARN_STREAM("COST MAP CENTER    @ (UTM):\t"<<utm_nort<<","<<utm_east);
        }

        /**
         * @brief  transform broadcaster function
         * @param  msg: the odom pose in UTM
         * @return void
         * @note   This function saves the odom pose in UTM
         */    
        void transformer(const nav_msgs::Odometry & msg){
            if (this->once_ == true){
                utm_odom.position.x = msg.pose.pose.position.x;
                utm_odom.position.y = msg.pose.pose.position.y;
                utm_odom.position.z = 0;
                utm_odom.orientation.x = 0; //msg.pose.pose.orientation.x;
                utm_odom.orientation.y = 0; //msg.pose.pose.orientation.y;
                utm_odom.orientation.z = 0; //msg.pose.pose.orientation.z;
                utm_odom.orientation.w = 1; //msg.pose.pose.orientation.w;
                ROS_WARN_STREAM("ODOM START @ (UTM):\t"<<utm_odom.position.x<<","<<utm_odom.position.y);

                this->once_ = false;
                ROS_ERROR("Restart odom_simulator if you want to reset ODOM START again!");
            }

        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom simulator");
    ros::NodeHandle n;
    

    Odom_sim hello(n);
    
    ros::Rate r(500);
    while(ros::ok() && hello.once_ == true){
        ros::spinOnce();
        r.sleep();
    }
    while(ros::ok()){
        hello.publish_tf();
        r.sleep();
    }

    return 0;
}