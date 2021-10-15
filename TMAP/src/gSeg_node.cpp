#include <iostream>
#define PCL_NO_PRECOMPILE

#include "TMAP/patchwork_obstacle.hpp"

boost::shared_ptr<Patchwork_M> patchwork_m;

ros::Publisher pub_nongroundpc;
ros::Publisher pub_groundpc;
ros::Publisher pub_obstaclepc;
ros::Publisher pub_LIG_Node;
ros::Publisher pub_gng;

std::string node_topic_;
std::string ground_filter_algorithm_;
std::string ptCloud_type_;

void callbackNode(const tMap_msgs::TrvMapDet_node::ConstPtr& node_msgs){
    tMap_msgs::TrvMapDet_node new_node_msg = *node_msgs;
    
    std::string cloud_frame;
    pcl::PointCloud<pcl::PointXYZI> pc_curr;
    if (ptCloud_type_ == "None"){
        pcl::fromROSMsg(new_node_msg.ptCloud_raw, pc_curr);
        cloud_frame = new_node_msg.header.frame_id;
    } else if (ptCloud_type_ == "GAlign"){
        pcl::fromROSMsg(new_node_msg.ptCloud_gAligned, pc_curr);
        cloud_frame = new_node_msg.ptCloud_gAligned.header.frame_id;
    } else {
        ROS_WARN("Please Select the Align Type among None / DeSkew / GAlign");
    }

    pcl::PointCloud<pcl::PointXYZI> pc_ground;
    pcl::PointCloud<pcl::PointXYZI> pc_nonground;
    pcl::PointCloud<pcl::PointXYZI> pc_obstacle;

    double patchwork_process_time;
    if(ground_filter_algorithm_ == "patchwork_m"){
        patchwork_m->estimate_ground(pc_curr, pc_ground, pc_nonground, patchwork_process_time);
        pc_obstacle = patchwork_m->get_obstacle_pc();
    } else{
        ROS_ERROR("Please set the algorithm name correctly");
    }
    std::cout << "\033[34;1m" << "Patchwork Process Time: " << patchwork_process_time << "\033[37;0m" << std::endl;

    // euigon
    pcl::PointCloud<pcl::PointXYZI> out;
    for (const auto &p : pc_nonground.points)
    {
        pcl::PointXYZI temp = p;
        temp.intensity = 1.0;
        out.points.push_back(temp);
    }
    for (const auto &p : pc_ground.points)
    {
        pcl::PointXYZI temp = p;
        temp.intensity = 0.0;
        out.points.push_back(temp);
    }

    new_node_msg.ptCloud_ground     = ptHandle::cloud2msg(pc_ground, cloud_frame);
    new_node_msg.ptCloud_nonground  = ptHandle::cloud2msg(pc_nonground, cloud_frame);
    new_node_msg.ptCloud_obstacle   = ptHandle::cloud2msg(pc_obstacle, cloud_frame);
    new_node_msg.ptCloud_ground.header = new_node_msg.header;
    new_node_msg.ptCloud_nonground.header = new_node_msg.header;
    new_node_msg.ptCloud_obstacle.header = new_node_msg.header;

    pub_groundpc.publish(new_node_msg.ptCloud_ground);
    pub_nongroundpc.publish(new_node_msg.ptCloud_nonground);
    pub_obstaclepc.publish(new_node_msg.ptCloud_obstacle);
    pub_LIG_Node.publish(new_node_msg);

    sensor_msgs::PointCloud2 out_cloud = ptHandle::cloud2msg(out, cloud_frame);
    out_cloud.header = new_node_msg.header;
    pub_gng.publish(out_cloud);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "patchwork");

    ros::NodeHandle nh;

    // set parameters
    nh.param<std::string>("/nodegen/node_topic", node_topic_, "/lig_node/pre_processing_node");
    nh.param<std::string>("/nodegen/ptCloud_type", ptCloud_type_, "None");
    nh.param<std::string>("/gSeg_algorithm", ground_filter_algorithm_, "patchwork");

    ROS_INFO_STREAM("TrvNode  Topic: "+ node_topic_);
    ROS_INFO_STREAM("Algorithm Name: "+ ground_filter_algorithm_);
    ROS_INFO_STREAM("PtCloud  Align: "+ ptCloud_type_);

    if(ground_filter_algorithm_ == "patchwork_m"){
        patchwork_m.reset(new Patchwork_M(&nh));
    } else{
        ROS_ERROR("Please set the algorithm name correctly");
    }

    ros::Subscriber sub_ptCloud = nh.subscribe<tMap_msgs::TrvMapDet_node>(node_topic_, 100, callbackNode, ros::TransportHints().tcpNoDelay());

    //Total ptClouds
    pub_LIG_Node  = nh.advertise<tMap_msgs::TrvMapDet_node> ("/lig_node/GSeged_node",1);
    //publish ptClouds in each
    pub_groundpc    = nh.advertise<sensor_msgs::PointCloud2> ("/GSeg/ground_ptCloud",1);
    pub_nongroundpc = nh.advertise<sensor_msgs::PointCloud2> ("/GSeg/nonground_ptCloud",1);
    pub_obstaclepc  = nh.advertise<sensor_msgs::PointCloud2> ("/GSeg/obstacle_ptCloud",1);
    
    pub_gng = nh.advertise<sensor_msgs::PointCloud2>("/GSeg/ground_nonground_ptCloud", 1);

    ros::spin();
    return 0;
}
