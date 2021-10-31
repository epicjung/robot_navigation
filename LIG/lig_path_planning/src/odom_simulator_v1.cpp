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
        ros::Subscriber sub_gps, sub_Odom2Map;
        bool is_slam_;
        
        tf2_ros::TransformBroadcaster tf2_map2odom, tf2_odom2basefootprint, tf2_map2pp_map;
        
        // global_utm_center (map)  : the center UTM position of the global costmap
        // global_utm_start (odom)  : the starting UTM position of the robot (when slam starts in SLAM mode or when GPS is first received in GPS mode)
        // current_pose (base_link) : the current UTM position of the robot
        geometry_msgs::Pose global_utm_center, global_utm_start, current_pose;

    public:
        bool once_ = true;
        /**
         *  @brief This function publishes the transform from pp_map to map
         */    
        void publish_tf(){
            // std::cout<<"publish_tf"<<std::endl;
            // tf2_map2pp_map.sendTransform(tf2::StampedTransform(tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)), ros::Time::now(), "map", "center"));
            
            // NOTE: **Odom2GlobalMap** map 2 pp_map (costmap center): parent->map , child->pp_map
            // tf::Quaternion quat1(global_utm_start.orientation.x, global_utm_start.orientation.y, global_utm_start.orientation.z, global_utm_start.orientation.w);
            // tf::Vector3 pos1(global_utm_start.position.x, global_utm_start.position.y, global_utm_start.position.z);
            // tf::Transform t_map = tf::Transform(quat1, pos1);
            tf2::Transform t_map = tf2::Transform( 
                            tf2::Quaternion(global_utm_start.orientation.x, global_utm_start.orientation.y, global_utm_start.orientation.z, global_utm_start.orientation.w),
                            tf2::Vector3(global_utm_start.position.x, global_utm_start.position.y, global_utm_start.position.z));

            // tf::Quaternion quat2(global_utm_center.orientation.x, global_utm_center.orientation.y, global_utm_center.orientation.z, global_utm_center.orientation.w);
            // tf::Vector3 pos2(global_utm_center.position.x, global_utm_center.position.y, global_utm_center.position.z);
            // tf::Transform t_pp_map = tf::Transform(quat2, pos2);
            tf2::Transform t_pp_map = tf2::Transform( 
                            tf2::Quaternion(global_utm_center.orientation.x, global_utm_center.orientation.y, global_utm_center.orientation.z, global_utm_center.orientation.w),
                            tf2::Vector3(global_utm_center.position.x, global_utm_center.position.y, global_utm_center.position.z));

            printf("map UTM start: (%f,%f)\npp_map UTM center: (%f,%f)\n", global_utm_start.position.x, global_utm_start.position.y, global_utm_center.position.x, global_utm_center.position.y);
            tf2::Transform mm = t_map.inverse();
            printf("t_map.inverse(): (%f,%f,%f,   %f,%f,%f,%f)\n", mm.getOrigin().x(), mm.getOrigin().y(), mm.getOrigin().z(), mm.getRotation().x(), mm.getRotation().y(), mm.getRotation().z(), mm.getRotation().w());
            tf2::Transform t_map_pp_map = mm * t_pp_map;
            printf("equal: (%f,%f,%f,   %f,%f,%f,%f)\n",t_map_pp_map.getOrigin().x(),t_map_pp_map.getOrigin().y(),t_map_pp_map.getOrigin().z(),t_map_pp_map.getRotation().x(),t_map_pp_map.getRotation().y(),t_map_pp_map.getRotation().z(),t_map_pp_map.getRotation().w());
            geometry_msgs::TransformStamped ts;
            ts.header.stamp = ros::Time::now();
            ts.header.frame_id= "map"; 
            ts.child_frame_id = "pp_map";
            ts.transform.translation.x = t_map_pp_map.getOrigin().x();
            ts.transform.translation.y = t_map_pp_map.getOrigin().y();
            ts.transform.translation.z = t_map_pp_map.getOrigin().z();
            ts.transform.rotation.x = t_map_pp_map.getRotation().x();
            ts.transform.rotation.y = t_map_pp_map.getRotation().y();
            ts.transform.rotation.z = t_map_pp_map.getRotation().z();
            ts.transform.rotation.w = t_map_pp_map.getRotation().w();
            tf2_map2pp_map.sendTransform(ts);
        }

        /**
         * @brief  Init
         */    
        Odom_sim(ros::NodeHandle & nh, bool is_slam): nh_(nh), is_slam_(is_slam){
            if (is_slam_){
                // Use SLAM as odom simulator
                std::cout<<"new Odom Sim object created with SLAM mode"<<std::endl;
                // this->sub_slam = this->nh_.subscribe("/receive_gps/odom", 400, &Odom_sim::slam_callback, this);
                // this->sub_slam_first_gps = this->nh_.subscribe("/receive_gps/first_odom", 1, &Odom_sim::slam_first_gps_callback, this);
                this->sub_Odom2Map = this->nh_.subscribe("/receive_gps/first_odom", 1, &Odom_sim::map2pp_map_TFtransformer, this);
            } else {
                // Use GPS as odom simulator
                std::cout<<"new Odom Sim object created with GPS mode"<<std::endl;
                this->sub_gps = this->nh_.subscribe("/tcpfix", 100, &Odom_sim::gps_callback, this);
            }

            // The Global GPS position is the Global costmap's center GPS position
            double utm_nort, utm_east;
            gps_common::UTM(36.3683644417,127.362330045,&utm_nort,&utm_east); // gps_common::LLtoUTM(l_lati,l_long,utm_nort,utm_east,"52N");
            global_utm_center.position.x = utm_nort;
            global_utm_center.position.y = utm_east;
            global_utm_center.position.z = 0;
            global_utm_center.orientation.x = 0;
            global_utm_center.orientation.y = 0;
            global_utm_center.orientation.z = 0;
            global_utm_center.orientation.w = 1;
            ROS_WARN_STREAM("COST MAP CENTER    @ (UTM):\t"<<utm_nort<<","<<utm_east);
        }

        /**
         * @brief  Get the current GPS location and publish
         */
        void gps_callback(const sensor_msgs::NavSatFix& msg){
            
            // frame_id is "gps"
            // sensor_msgs::NavSatFix t_msg = msg;
            double l_lati = msg.latitude;
            double l_long = msg.longitude;
            // lat: 36.XXXX, long: 127.XXXX 
            // printf("lat: %lf, long: %lf\n",l_lati, l_long);
            
            double utm_nort, utm_east;
            // gps_common::LLtoUTM(l_lati,l_long,utm_nort,utm_east,"52N");
            gps_common::UTM(l_lati,l_long,&utm_nort,&utm_east);

            static bool once = true;
            if (once == true){
                global_utm_start.position.x = utm_nort;
                global_utm_start.position.y = utm_east;
                global_utm_start.position.z = msg.altitude;
                once = false;
            }

            this->current_pose.position.x = utm_nort;
            this->current_pose.position.y = utm_east;
            this->current_pose.position.z = msg.altitude;

            geometry_msgs::TransformStamped ts;
            ts.header.stamp = msg.header.stamp;
            ts.header.frame_id= "pp_odom"; 
            ts.child_frame_id = "pp_base_link";
            ts.transform.translation.x = utm_nort - global_utm_start.position.x;
            ts.transform.translation.y = utm_east - global_utm_start.position.y;
            ts.transform.translation.z = msg.altitude - global_utm_start.position.z;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
            ts.transform.rotation.w = 1.0;
            this->tf2_odom2basefootprint.sendTransform(ts);

            geometry_msgs::TransformStamped tss;
            tss.header.stamp = msg.header.stamp;
            tss.header.frame_id= "pp_map";
            tss.child_frame_id = "pp_odom";
            tss.transform.translation.x = global_utm_center.position.x-global_utm_start.position.x;
            tss.transform.translation.y = global_utm_center.position.y-global_utm_start.position.y;
            tss.transform.translation.z = 0.0;
            tss.transform.rotation.x = 0.0;
            tss.transform.rotation.y = 0.0;
            tss.transform.rotation.z = 0.0;
            tss.transform.rotation.w = 1.0;
            this->tf2_map2odom.sendTransform(tss);
        }

        void map2pp_map_TFtransformer(const nav_msgs::Odometry & msg){
            if (this->once_ == true){
                global_utm_start.position.x = msg.pose.pose.position.x;
                global_utm_start.position.y = msg.pose.pose.position.y;
                global_utm_start.position.z = 0;
                global_utm_start.orientation.x = 0; //msg.pose.pose.orientation.x;
                global_utm_start.orientation.y = 0; //msg.pose.pose.orientation.y;
                global_utm_start.orientation.z = 0; //msg.pose.pose.orientation.z;
                global_utm_start.orientation.w = 1; //msg.pose.pose.orientation.w;
                ROS_WARN_STREAM("ODOM START @ (UTM):\t"<<global_utm_start.position.x<<","<<global_utm_start.position.y);

                this->once_ = false;
                ROS_ERROR("Restart odom_simulator if you want to reset ODOM START again!");
            }

        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "odom simulator with gps");
    ros::NodeHandle n;
    
    std::string mode(argv[1]);
    std::string slam_mode("slam");    
    bool is_slam = (mode == slam_mode);

    Odom_sim hello(n, is_slam);
    // ros::spin();
    ros::Rate r(2);
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