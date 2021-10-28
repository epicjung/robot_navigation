 
/**
 * Created by Sumin on 2021/04/30
 * README *
 * @author: Sumin Hu
 * @github: 
 * @describe: LIG PATH PLANNING 
 * 
*/


#include <algorithm>
#include <vector>
#include <string>
#include <iostream>

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>
// #include <nav_core/base_global_planner.h>
// #include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
// #include <nav_msgs/GetPlan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_broadcaster.h>
#include <gps_common/conversions.h>

// #include <pluginlib/class_loader.hpp>


#ifndef __LIG_PATH_H__
#define __LIG_PATH_H__

class LIGPATH {
public:
    LIGPATH(ros::NodeHandle & nh){
        std::cout<<"new LIGPATH object created"<<std::endl;
        this->gps_sub = this->nh_.subscribe("/tcpfix", 100, &LIGPATH::gps_callback, this);
        this->goal_sub1 = this->nh_.subscribe("/goal1", 1, &LIGPATH::goal_callback1, this);
        this->goal_sub2 = this->nh_.subscribe("/goal2", 1, &LIGPATH::goal_callback2, this);
        this->goal_sub2 = this->nh_.subscribe("/goal2", 1, &LIGPATH::goal_callback2, this);
        this->imu_sub = this->nh_.subscribe("/imu/data",100,&LIGPATH::imu_callback, this);
    };
    ~LIGPATH(){}
    /** 
    * @describe: Run DBSCAN clustering alogrithm
    * @param: V {std::vector<T>} : data
    * @param: dim {unsigned int} : dimension of T (a vector-like struct)
    * @param: eps {Float} : epsilon or in other words, radian
    * @param: min {unsigned int} : minimal number of points in epsilon radian, then the point is cluster core point
    * @param: disfunc {DistanceFunc} : !!!! only used in bruteforce mode. Distance function recall. Euclidian distance is recommanded, but you can replace it by any metric measurement function
    * @usage: Object.Run() and get the cluster and noise indices from this->Clusters & this->Noise.
    * @pitfall: If you set big eps(search range) and huge density V, then kdtree will be a bottleneck of performance
    * @pitfall: You MUST ensure the data's identicality (TVector* V) during Run(), because DBSCAN just use the reference of data passed in.
    * @TODO: customize kdtree algorithm or rewrite it ,stop further searching when minimal number which indicates cluster core point condition is satisfied
    */

    int run();

    /**
     * @brief  occupancy map of 2dCostMap is retrieved. 
     * @param  goal The goal to plan to
     * @param  plan Will be filled in with the plan made by the planner
     * @return  True if planning succeeds, false otherwise
     */
    void occupancy_callback();

    /**
     * @brief  Get the current imu values. 400 Hz
     */
    void imu_callback(const sensor_msgs::Imu & msg);

    /**
     * @brief  Get the current GPS location. 5 Hz
     */
    void gps_callback(const sensor_msgs::NavSatFix & msg);
    
    /**
     * @brief  Set the initial static global goal, loaded from a static file.
     * @param  msg config/static_goals.gps에 있는 기본 lat-long을 rviz.launch에서 로딩함.
     * @return  성공적으로 로딩되면 rviz에서 "goal"이 띄워짐.
     */
    void goal_callback1(const sensor_msgs::NavSatFix & msg);

    /**
     * @brief  Set the global goal
     * @param  msg RVIZ에서 "2D Nav Goal"로 goal을 설정했을 때의 position임.
     *              Q. 여기서 나오는 position은 relative하게 나오는데 어떤 frame기준으로 나오는 건가? 
     *              A. RVIZ에서 "Global Options"의 "Fixed Frame" 기준으로 나옴.
     *                  예제 START -------------------
     *                      header: 
     *                        seq: 2
     *                        stamp: 
     *                          secs: 1627438418
     *                          nsecs:  16825010
     *                        frame_id: "base_link"
     *                      pose: 
     *                        position: 
     *                          x: 31.1108112335
     *                          y: 42.4342880249
     *                          z: 0.0
     *                        orientation: 
     *                          x: 0.0
     *                          y: 0.0
     *                          z: 0.0474418065235
     *                          w: 0.998874003563
     *                  예제 END-------------------
     * @return  
     */
    void goal_callback2(const geometry_msgs::PoseStamped & msg);
    
   void local_planning();

private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub, goal_sub1, goal_sub2, imu_sub;

    tf2_ros::TransformBroadcaster tf2_here2goal;

    geometry_msgs::Pose global_utm_start;
    geometry_msgs::Pose global_goal;
    
    // initial position and goal position in lat-long? OR UTM?
    geometry_msgs::Pose init_pose, goal_pose, current_pose; 

    // void hello();
    // pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    // pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
    // pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
};


class LocalPlanner : public nav_core::BaseLocalPlanner {

public: 
    LocalPlanner();
    LocalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initalize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(  const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, 
                    std::vector<geometry_msgs::PoseStamped>& plan
                );

 


};



#endif  //__LIG_PATH_H__
