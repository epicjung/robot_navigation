// path_node.cpp
#include <ros/ros.h>
#include "lig_path.h"


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "path_node");
    ros::NodeHandle nh("~");

    printf("argument 1: %s\nargument 2: %s\nargument 3: %s\n total of %d arguments\n",argv[0], argv[1], argv[2], argc);

    LIGPATH lig_path(nh);

    ros::spin();
    // ros::Rate loop_rate(0.1); 
    // while (ros::ok()){
    //     ros::spinOnce();
    //     std::cout<<"sleeping ..."<<std::endl;
    //     loop_rate.sleep();
    // }

}