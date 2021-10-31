
#include "lig_path.h"

LIGPATH::LIGPATH(ros::NodeHandle & nh): nh_(nh)
{

    std::cout<<"new LIGPATH object created"<<std::endl;
    this->gps_sub = this->nh_.subscribe("/tcpfix", 100, &LIGPATH::gps_callback, this);
    this->goal_sub1 = this->nh_.subscribe("/goal1", 1, &LIGPATH::goal_callback1, this);
    this->goal_sub2 = this->nh_.subscribe("/goal2", 1, &LIGPATH::goal_callback2, this);
    this->goal_sub2 = this->nh_.subscribe("/goal2", 1, &LIGPATH::goal_callback2, this);
    this->imu_sub = this->nh_.subscribe("/imu/data",100,&LIGPATH::imu_callback, this);

}

void LIGPATH::imu_callback(const sensor_msgs::Imu & msg){

    // get initial heading direction of robot
    // IMU 센서가 robot의 heading 방향과 일치하는지 확인은 해야함!
    static bool once = true;
    if (global_utm_start.position.x != 0 && once == true){
        global_utm_start.orientation.x = msg.orientation.x;
        global_utm_start.orientation.y = msg.orientation.y;
        global_utm_start.orientation.z = msg.orientation.z;
        global_utm_start.orientation.w = msg.orientation.w;
        once = false;
    }

    this->current_pose.orientation.x = msg.orientation.x;
    this->current_pose.orientation.y = msg.orientation.y;
    this->current_pose.orientation.z = msg.orientation.z;
    this->current_pose.orientation.w = msg.orientation.w;

}

void LIGPATH::gps_callback(const sensor_msgs::NavSatFix & msg){
    // frame_id is "gps"
    // sensor_msgs::NavSatFix t_msg = msg;
    double l_lati = msg.latitude;
    double l_long = msg.longitude;
    // lat: 36.XXXX, long: 127.XXXX
    printf("lat: %lf, long: %lf\n",l_lati, l_long);
    
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
    ts.header.frame_id= "map";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = utm_nort - global_utm_start.position.x;
    ts.transform.translation.y = utm_east - global_utm_start.position.y;
    ts.transform.translation.z = msg.altitude - global_utm_start.position.z;
    ts.transform.rotation.x = current_pose.orientation.x;
    ts.transform.rotation.y = current_pose.orientation.y;
    ts.transform.rotation.z = current_pose.orientation.z;
    ts.transform.rotation.w = current_pose.orientation.w;

    this->tf2_here2goal.sendTransform(ts);
}

void LIGPATH::goal_callback1(const sensor_msgs::NavSatFix & msg){
    // frame_id is "goal"; check 

    // lat: 36.XXXX, long: 127.XXXX
    printf("lat: %lf, long: %lf\n",msg.latitude, msg.longitude);

    double l_lati = msg.latitude;
    double l_long = msg.longitude;
    double utm_nort, utm_east;
    gps_common::UTM(l_lati,l_long,&utm_nort,&utm_east);

    goal_pose.position.x = utm_nort;
    goal_pose.position.y = utm_east;
    goal_pose.position.z = 100;
    goal_pose.orientation.x = 0;
    goal_pose.orientation.y = 0;
    goal_pose.orientation.z = 0;
    goal_pose.orientation.w = 1;


}

void LIGPATH::goal_callback2(const geometry_msgs::PoseStamped & msg){
    // frame_id is "$(rviz's fixed frame now)", which is most likely "map"
    // geometry_msgs::PoseStamped t_msg = msg;

    if (msg.header.frame_id != "/map" || msg.header.frame_id != "/base_link"){

    }
    // global_goal
    // std::cout<< t_msg << std::endl;
    
    // t_msg.pose.position.x;
    // t_msg.pose.position.y;
    // t_msg.pose.position.z;
    // t_msg.pose.orientation.x;
    // t_msg.pose.orientation.y;
    // t_msg.pose.orientation.z;
    // t_msg.pose.orientation.w;

}


void LIGPATH::local_planning(){
    std::cout<<"hello"<<std::endl;
}