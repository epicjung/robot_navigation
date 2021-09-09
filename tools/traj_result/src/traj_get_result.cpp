#include <iostream>
#include <fstream>
#include <cstdio>
#include <experimental/filesystem>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;

// https://github.com/uzh-rpg/rpg_trajectory_evaluation#prepare-the-data

// rosrun rpg_trajectory_evaluation analyze_trajectory_single.py <result_folder>

std::string SAVE_DIR = "/home/euigon/lvi_sam_working/src/tools/traj_result/";
std::string output_filename = "results/";
nav_msgs::Path trajectory_msg;

#include <signal.h>

void signal_handler(sig_atomic_t s){

    std::cout<<"\033[1;32m"<< "You pressed Ctrl+C..." <<"\033[0m"<<std::endl;
    std::cout<<"\033[1;32m"<< "Saving..." <<"\033[0m"<<std::endl;

    std::vector<geometry_msgs::PoseStamped> poses = trajectory_msg.poses;

    std::ofstream sc_output(output_filename, std::ios::app);

    for (geometry_msgs::PoseStamped pose_it : poses){

        std_msgs::Header header = pose_it.header;
        geometry_msgs::Point tl = pose_it.pose.position;
        geometry_msgs::Quaternion qt = pose_it.pose.orientation;

        sc_output<<header.stamp<<" "<<tl.x<<" "<<tl.y<<" "<<tl.z<< " "<<qt.x <<" "<< qt.y<<" "<<qt.z<<" "<<qt.w;
        sc_output<<std::endl;
    }

    sc_output.close();
    std::cout<<"\033[1;32m"<< "Done..." <<"\033[0m"<<std::endl;

    exit(1);
}

void callbackNode(const nav_msgs::Path::ConstPtr& traj_msg){
    ROS_INFO("\033[1;32m----> receive trajectory_topic.\033[0m");
    trajectory_msg = *traj_msg;
    std::cout << "data num "<<trajectory_msg.poses.size() << std::endl;

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TrajResult", ros::init_options::AnonymousName);
    //
    ros::NodeHandle nh;
    
    signal(SIGINT,signal_handler); //exit program when to press ctrl+c

    std::string filename = argv[1];

    output_filename = SAVE_DIR + filename;
    ROS_INFO_STREAM("folder_name " + output_filename);
    
    // For initialization
    std::ofstream sc_output(output_filename);
    sc_output.close();

    ros::Subscriber NodeSubscriber = nh.subscribe<nav_msgs::Path>("/trajectory", 100,callbackNode);

    ros::spin();

    return 0;
}
