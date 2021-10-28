#include "lig_path.h"
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  
  std::cout <<"1: "   << argv[1]                // hello
            <<"\n2: " << argv[2]                // 1
            <<"\n3: " << argv[3]                // 2
            <<"\n4: " << argv[4]  <<std::endl;  // 3

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  lig_move_base::LIGMoveBase move_base( buffer );

  ros::spin();

  return(0);
}