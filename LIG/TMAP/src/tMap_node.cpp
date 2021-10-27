#include <iostream>
#define PCL_NO_PRECOMPILE

#include "TMAP/traversability_map.hpp"

boost::shared_ptr<RoughTerrain_tMap> tMap;

ros::Publisher pub_tMap;
ros::Publisher pub_tMap_ground;
ros::Publisher pub_tMap_obstacle;

void callbackPtCloudMsg(const tMap_msgs::TrvMapDet_node::ConstPtr& node_msg){
    std::chrono::system_clock::time_point s = std::chrono::system_clock::now();

    tMap->determine_tMap(*node_msg);

    std::chrono::duration<double> sec = std::chrono::system_clock::now() - s;
    std::cout<<"\033[34;1m"<<"tMapGen Process: " << sec.count() << " seconds" <<  "\033[32;0m" << std::endl;
    std::cout<<" "<<std::endl;

    nav_msgs::OccupancyGrid travel_map_obstacle = tMap->get_occupancy_grid_map();
    nav_msgs::OccupancyGrid travel_map_ground   = tMap->get_occupancy_grid_ground_map();
    nav_msgs::OccupancyGrid tMap_layers   = tMap->get_occupancy_grid_tMap();

    pub_tMap.publish(tMap_layers);
    pub_tMap_obstacle.publish(travel_map_obstacle);
    pub_tMap_ground.publish(travel_map_ground);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tMap");

    ros::NodeHandle nh;

    pub_tMap = nh.advertise<nav_msgs::OccupancyGrid>("/traversability_map",1);
    pub_tMap_obstacle= nh.advertise<nav_msgs::OccupancyGrid>("/traversability_obstacle_map",1);
    pub_tMap_ground  = nh.advertise<nav_msgs::OccupancyGrid>("/traversability_ground_map",1);
    
    ros::Subscriber sub_PtClouds = nh.subscribe<tMap_msgs::TrvMapDet_node>("/lig_node/GSeged_node", 1, callbackPtCloudMsg, ros::TransportHints().tcpNoDelay());

    tMap.reset(new RoughTerrain_tMap(&nh));
    ros::spin();

    return 0;
}