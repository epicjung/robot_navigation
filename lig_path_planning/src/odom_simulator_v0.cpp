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
        ros::Subscriber sub_gps, sub_slam, sub_slam_first_gps, sub_Odom2Map;
        ros::Publisher pub;
        bool is_slam_;
        tf2_ros::TransformBroadcaster tf2_map2odom, tf2_odom2basefootprint, tf2_sumin2other, tf2_map2center;
        
        // global_utm_center (map)  : the center UTM position of the global costmap
        // global_utm_start (odom)  : the starting UTM position of the robot (when slam starts in SLAM mode or when GPS is first received in GPS mode)
        // current_pose (base_link) : the current UTM position of the robot
        geometry_msgs::Pose global_utm_center, global_utm_start, current_pose;

        // initial position and goal position in lat-long? OR UTM?
        // geometry_msgs::Pose init_pose, goal_pose,
    public:

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

        void slam_callback(const nav_msgs::Odometry & msg){
            printf("got slam msg\n");
            // geometry_msgs::TransformStamped ts;
            // ts.header.stamp = msg.header.stamp;
            // ts.header.frame_id= "pp_odom";
            // ts.child_frame_id = "pp_base_link";
            // ts.transform.translation.x = msg.pose.pose.position.x;
            // ts.transform.translation.y = msg.pose.pose.position.y;
            // ts.transform.translation.z = 0.0;
            // ts.transform.rotation.x = 0.0;
            // ts.transform.rotation.y = 0.0;
            // ts.transform.rotation.z = 0.0;
            // ts.transform.rotation.w = 1.0;
            // this->tf2_odom2basefootprint.sendTransform(ts);

            // // Just returning all the time although this never changes because the refresh rate is important
            // geometry_msgs::TransformStamped tss;
            // tss.header.stamp = msg.header.stamp;
            // tss.header.frame_id= "pp_map";     // cost map center utm
            // tss.child_frame_id = "pp_odom";    // slam starting utm
            // tss.transform.translation.x = global_utm_center.position.x-global_utm_start.position.x;
            // tss.transform.translation.y = global_utm_center.position.y-global_utm_start.position.y;
            // tss.transform.translation.z = 0.0;
            // tss.transform.rotation.x = 0.0;
            // tss.transform.rotation.y = 0.0;
            // tss.transform.rotation.z = 0.0;
            // tss.transform.rotation.w = 1.0;
            // this->tf2_map2odom.sendTransform(tss);
            
            // geometry_msgs::TransformStamped tsss;
            // tsss.header.stamp = msg.header.stamp;
            // tsss.header.frame_id= "pp_odom";     // cost map center utm
            // tsss.child_frame_id = "odom";    // slam starting utm
            // tsss.transform.translation.x = 0.0;
            // tsss.transform.translation.y = 0.0;
            // tsss.transform.translation.z = 0.0;
            // tsss.transform.rotation.x = 0.0;
            // tsss.transform.rotation.y = 0.0;
            // tsss.transform.rotation.z = 0.0;
            // tsss.transform.rotation.w = 1.0;
            // this->tf2_sumin2other.sendTransform(tsss);

            
            // Just returning all the time although this never changes because the refresh rate is important
            geometry_msgs::TransformStamped tss;
            
            // OPTION 1 and OPTION 2
            tss.header.stamp = msg.header.stamp;
            tss.header.stamp = ros::Time::now(); 

            tss.header.frame_id= "pp_map";     // cost map center utm
            tss.child_frame_id = "pp_odom";    // slam starting utm
            tss.transform.translation.x = global_utm_center.position.x-global_utm_start.position.x;
            tss.transform.translation.y = global_utm_center.position.y-global_utm_start.position.y;
            tss.transform.translation.z = 0.0;
            tss.transform.rotation.x = 0.0;
            tss.transform.rotation.y = 0.0;
            tss.transform.rotation.z = 0.0;
            tss.transform.rotation.w = 1.0;
            this->tf2_map2odom.sendTransform(tss);

            geometry_msgs::TransformStamped ts;
            
            // OPTION 1 and OPTION 2
            tss.header.stamp = msg.header.stamp;
            tss.header.stamp = ros::Time::now();

            ts.header.frame_id= "pp_odom";
            ts.child_frame_id = "pp_base_link";
            ts.transform.translation.x = msg.pose.pose.position.x;
            ts.transform.translation.y = msg.pose.pose.position.y;
            ts.transform.translation.z = 0.0;
            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
            ts.transform.rotation.w = 1.0;
            this->tf2_odom2basefootprint.sendTransform(ts);

            
            geometry_msgs::TransformStamped tsss;
            
            // OPTION 1 and OPTION 2
            tss.header.stamp = msg.header.stamp;
            tss.header.stamp = ros::Time::now();

            tsss.header.frame_id= "pp_odom";     // cost map center utm
            tsss.child_frame_id = "world";    // slam starting utm
            tsss.transform.translation.x = 0.0;
            tsss.transform.translation.y = 0.0;
            tsss.transform.translation.z = 0.0;
            tsss.transform.rotation.x = 0.0;
            tsss.transform.rotation.y = 0.0;
            tsss.transform.rotation.z = 0.0;
            tsss.transform.rotation.w = 1.0;
            this->tf2_sumin2other.sendTransform(tsss);

        }

        void slam_first_gps_callback(const nav_msgs::Odometry & msg){
            // printf("b\n");
            static bool once = true;
            if (once == true){
                global_utm_start.position.x = msg.pose.pose.position.x;
                global_utm_start.position.y = msg.pose.pose.position.y;
                global_utm_start.orientation.x = 0;
                once = false;
            }
            ROS_WARN_STREAM("ODOM START         @ (UTM):\t"<<global_utm_start.position.x<<","<<global_utm_start.position.y);
        }

        void map2pp_map_TFtransformer(const nav_msgs::Odometry & msg){
            static bool once = true;
            if (once == true){
                global_utm_start.position.x = msg.pose.pose.position.x;
                global_utm_start.position.y = msg.pose.pose.position.y;
                global_utm_start.orientation.x = 0;
                ROS_WARN_STREAM("ODOM START         @ (UTM):\t"<<global_utm_start.position.x<<","<<global_utm_start.position.y);

                // NOTE: **Odom2GlobalMap** map 2 pp_map (costmap center): parent->map , child->pp_map
                tf::Quaternion quat1(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
                tf::Vector3 pos1(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
                tf::Transform t_world_map = tf::Transform(quat1, pos1);
                
                tf::Quaternion quat2(global_utm_center.orientation.x, global_utm_center.orientation.y, global_utm_center.orientation.z, global_utm_center.orientation.w);
                tf::Vector3 pos2(global_utm_center.position.x, global_utm_center.position.y, global_utm_center.position.z);
                tf::Transform t_world_center = tf::Transform(quat2, pos2);

                tf::Transform t_map_center = t_world_map.inverse() * t_world_center;

                geometry_msgs::TransformStamped ts;
                ts.header.stamp = msg.header.stamp;
                ts.header.frame_id= "map"; 
                ts.child_frame_id = "pp_map";
                ts.transform.translation.x = t_map_center.getOrigin().x();
                ts.transform.translation.y = t_map_center.getOrigin().y();
                ts.transform.translation.z = t_map_center.getOrigin().z();
                ts.transform.rotation.x = t_map_center.getRotation().x();
                ts.transform.rotation.y = t_map_center.getRotation().y();
                ts.transform.rotation.z = t_map_center.getRotation().z();
                ts.transform.rotation.w = t_map_center.getRotation().w();

                ros::Rate r(1);
                while(ros::ok()){
                    this->tf2_map2center.sendTransform(ts);
                    r.sleep();
                }

                once = false;
                ROS_ERROR("Restart odom_simulator if you want to use it again!");
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
    ros::spin();
    return 0;
}