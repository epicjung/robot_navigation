// 
/*
 *  tf_repub.cpp
 *  This is to republish the /tf topic from a rosbag file
 *  Input: 
 *  Output: 
 *      header.stamp = ros::Time::now()
 * */

#include <algorithm>
#include <vector>
#include <string>
#include <iostream>
#include <map>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;

class Realtime_tf_publisher {
    private:
        ros::NodeHandle nh_;    
        ros::Subscriber sub_;
        // vector<tf2_ros::TransformBroadcaster> tfs_;
        // vector<geometry_msgs::TransformStamped> tf_msg_;
        map<string, pair<tf2_ros::TransformBroadcaster, geometry_msgs::TransformStamped> > tf_map_;

        bool once_ = true;
    public:

        /**
         * @brief  Constructor
         */    
        Realtime_tf_publisher(ros::NodeHandle & nh): nh_(nh){
            sub_ = nh_.subscribe("/tf_orig", 1, &Realtime_tf_publisher::transformer, this);
        }    

        /**
         * @brief  transform broadcaster function
         * @param  msg: the odom pose in UTM
         * @return void
         * @note   This function saves the odom pose in UTM
         */    
        void transformer(const tf2_msgs::TFMessage & msg){

            for(int i = 0; i < msg.transforms.size(); i++){
                string source2target = msg.transforms[i].header.frame_id + string("2") +msg.transforms[i].child_frame_id;
                
                if (tf_map_.find(source2target) == tf_map_.end()){
                    tf_map_.emplace(source2target, make_pair(tf2_ros::TransformBroadcaster(), msg.transforms[i]));
                }

                tf_map_[source2target].second = msg.transforms[i];
                tf_map_[source2target].second.header.stamp = ros::Time::now();
            }

            for (auto it = tf_map_.begin(); it != tf_map_.end(); it++){
                it->second.first.sendTransform(it->second.second);
            }
            
        }


};

int main(int argc, char **argv){
    // reference: https://answers.ros.org/question/9396/renaming-a-topic-inside-a-bag-file/
    // ROS_INFO("If you want to re-publish topic /tf, first you must rename this topic in your rosbag file.\n To do so, run `rosrun rosbag topic_renamer.py /tf ligbag3_test.bag /tf_orig ligbag3_newtest.bag`");
    ros::init(argc, argv, "tf_re_simulator");
    ros::NodeHandle n;
    
    Realtime_tf_publisher republish(n);
    
    ros::spin();

    return 0;
}