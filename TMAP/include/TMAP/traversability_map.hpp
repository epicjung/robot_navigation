#ifndef PATCHWORK_TMAP_H
#define PATCHWROK_TMAP_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "tMap_msgs/TrvMapDet_node.h"

#include "../../utils/ptcloud_handle.hpp"
#include "../../utils/map_handle.hpp"

struct nodeInfo{
    pcl::PointCloud<pcl::PointXYZI> ptCloud_ground;
    pcl::PointCloud<pcl::PointXYZI> ptCloud_obstacle;
    pcl::PointCloud<pcl::PointXYZI> ptCloud_dynamic;
    geometry_msgs::Pose lidar_pose;
};

class RoughTerrain_tMap{
public:
    RoughTerrain_tMap(){};
    RoughTerrain_tMap(ros::NodeHandle* nh):node_handle_(*nh){
        ROS_INFO("tMap generator");

        node_handle_.param<double>("tMap/max_range", ptRange_max_, 40.0);
        node_handle_.param<double>("tMap/min_range", ptRange_min_, 2.0);
        node_handle_.param<double>("tMap/map_resolution", map_res_, 0.01);
        node_handle_.param<double>("tMap/odom_diff_threshold", odom_diff_threshold_, 1);
        node_handle_.param("tMap/map_window_size", map_window_size_, 100);
        node_handle_.param<bool>("tMap/margin_node", margin_node_, true);
        node_handle_.param<bool>("tMap/updateMap", updateMap_, true);

        xMap_min_ = -ptRange_max_;
        yMap_min_ = -ptRange_max_;
        xMap_max_ = +ptRange_max_;
        yMap_max_ = +ptRange_max_;
    }
    ~RoughTerrain_tMap(){}

    void determine_tMap(const tMap_msgs::TrvMapDet_node node_msg);
    nav_msgs::OccupancyGrid get_occupancy_grid_map(){
        return tMap_obstacle_;
    }
    nav_msgs::OccupancyGrid get_occupancy_grid_ground_map(){
        return tMap_ground_;
    }
    nav_msgs::OccupancyGrid get_occupancy_grid_tMap(){
        return tMap_;
    }

private:
    std::deque<nodeInfo> node_vec_;
    nodeInfo node_;
    double map_res_;
    double ptRange_max_;
    double ptRange_min_;
    double xMap_max_, xMap_min_, yMap_max_, yMap_min_;
    double odom_diff_threshold_;

    int map_window_size_;
    bool updateMap_;
    bool margin_node_;
    ros::NodeHandle node_handle_;

    nav_msgs::OccupancyGrid tMap_obstacle_;
    nav_msgs::OccupancyGrid tMap_ground_;
    nav_msgs::OccupancyGrid tMap_;

    nav_msgs::OccupancyGrid getOccuObstacle(double resolutinon);
    nav_msgs::OccupancyGrid getOccuGround(double resolution);

    nav_msgs::OccupancyGrid setMapLayers(double resolution);

    bool updateMap(nodeInfo &new_node);
};

bool RoughTerrain_tMap::updateMap(nodeInfo &new_node){
    if (node_vec_.size() ==0) return true;

    nodeInfo last_node = node_vec_.back();

    double dist2D_sqr = (last_node.lidar_pose.position.x - new_node.lidar_pose.position.x)*(last_node.lidar_pose.position.x - new_node.lidar_pose.position.x) + (last_node.lidar_pose.position.y - new_node.lidar_pose.position.y)*(last_node.lidar_pose.position.y - new_node.lidar_pose.position.y);
    
    if (dist2D_sqr < odom_diff_threshold_) {
        return false;
    } else {
        return true;
    }
}

void RoughTerrain_tMap::determine_tMap(const tMap_msgs::TrvMapDet_node node_msg){
    //Input: pointcloud for ground and nonground 
    //Output:traversable map for global frame

    nodeInfo new_node;
    new_node.ptCloud_ground = ptHandle::cloudmsg2cloud<pcl::PointXYZI>(node_msg.ptCloud_ground);
    new_node.ptCloud_obstacle = ptHandle::cloudmsg2cloud<pcl::PointXYZI>(node_msg.ptCloud_obstacle);
    new_node.lidar_pose = node_msg.lidar_odom2D;

    if ( updateMap(new_node) ){
        node_vec_.push_back(new_node);
    } else{
        updateMap_ = false;
    }

    if ( margin_node_ && ((int)node_vec_.size() > map_window_size_) ){
        node_vec_.pop_front();
    }
    
    std::cout << "\033[34;1m" << "Size of node vector: " << node_vec_.size() << "\033[32;0m"<<std::endl;
    
    tMap_ = setMapLayers(map_res_);

    tMap_obstacle_.header = node_msg.header;
    tMap_ground_.header = node_msg.header;
    tMap_.header = node_msg.header;

    tMap_obstacle_.header.frame_id = "map";
    tMap_ground_.header.frame_id = "map";
    tMap_.header.frame_id = "map";
}

nav_msgs::OccupancyGrid RoughTerrain_tMap::setMapLayers(double resolution){
    ROS_INFO("Set Map");
    nav_msgs::OccupancyGrid occugrid;

    // Set map size
    // FIND MIN, MAX point of map
    for (int i = 0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_lidar = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_obstacle);
        cv::Mat tmp_ptCloud = cur_pose * cur_lidar;

        double min, max;
        cv::Point minPt, maxPt;

        // Find Min & Max value in X-axis
        cv::minMaxLoc(tmp_ptCloud.row(0), &min, &max, &minPt, &maxPt);
        xMap_min_ = (min < xMap_min_)? min : xMap_min_;
        xMap_max_ = (max > xMap_max_)? max : xMap_max_;

        // Find Min & Max value in Y-axis
        cv::minMaxLoc(tmp_ptCloud.row(1), &min, &max, &minPt, &maxPt);
        yMap_min_ = (min < yMap_min_)? min : yMap_min_;
        yMap_max_ = (max > yMap_max_)? max : yMap_max_;
    }
    
    cv::Point original_pt = cv::Point(-xMap_min_/resolution, -yMap_min_/resolution);
    
    cv::Mat T_Global2Pix = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    T_Global2Pix.at<float>(0,0) = 1./resolution;
    T_Global2Pix.at<float>(1,1) = 1./resolution;
    T_Global2Pix.at<float>(2,2) = 1;
    T_Global2Pix.at<float>(3,3) = 1;
    T_Global2Pix.at<float>(0,3) = -xMap_min_ / resolution;
    T_Global2Pix.at<float>(1,3) = -yMap_min_ / resolution;
    std::cout << "\033[33;1m" << "x Max: " << xMap_max_ << " || x Min: " << xMap_min_ << " || y Max: " << yMap_max_ << " || y Min: " << yMap_min_ << "\033[32;0m"<< std::endl;
    
    // Layers
    cv::Mat occu_ground_map = cv::Mat(cv::Size((xMap_max_-xMap_min_)/resolution+1,(yMap_max_-yMap_min_)/resolution+1), CV_8UC1, 120);
    cv::Mat occu_obstacle_map = cv::Mat(cv::Size((xMap_max_-xMap_min_)/resolution+1,(yMap_max_-yMap_min_)/resolution+1), CV_8UC1, 1);

    for(int i=0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_groundPtCloud = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_ground);
        cv::Mat cur_ObstaclePtCloud = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_obstacle);

        cv::Mat tmp_groundptPix = T_Global2Pix * cur_pose * cur_groundPtCloud;
        cv::Mat tmp_obstacleptPix = T_Global2Pix * cur_pose * cur_ObstaclePtCloud;

        cv::Mat tmp_ptPix_c = T_Global2Pix * cur_pose;
        cv::Point tmpcvPt_c = cv::Point(tmp_ptPix_c.at<float>(0,3), tmp_ptPix_c.at<float>(1,3));

        if (tmpcvPt_c.x < 0 || tmpcvPt_c.x >= occu_obstacle_map.size().width || tmpcvPt_c.y < 0 || tmpcvPt_c.y >= occu_obstacle_map.size().height){
            // If pose of robot (lidar) is out of occupancy grid map region.
            continue;
        } else {
            // ground map
            for (int k = 0; k<tmp_groundptPix.size().width; k++){
                cv::Point tmpcvPt1 = cv::Point(tmp_groundptPix.at<float>(0,k), tmp_groundptPix.at<float>(1,k));
                if(tmpcvPt1.x < 0|| tmpcvPt1.x >= occu_ground_map.size().width ||tmpcvPt1.y < 0 || tmpcvPt1.y >= occu_ground_map.size().height) continue;
                occu_ground_map.at<unsigned char>(tmpcvPt1.y, tmpcvPt1.x) = 255;
            }

            // obstacle map
            for (int k = 0; k<tmp_obstacleptPix.size().width; k++){
                cv::Point tmpcvPt1 = cv::Point(tmp_obstacleptPix.at<float>(0,k), tmp_obstacleptPix.at<float>(1,k));
                if(tmpcvPt1.x < 0|| tmpcvPt1.x >= occu_obstacle_map.size().width ||tmpcvPt1.y < 0 || tmpcvPt1.y >= occu_obstacle_map.size().height) {
                    // If 2D location of point from pointcloud in cv map coordinate is out of occupancy grid map region.
                    continue;
                }
                occu_obstacle_map.at<unsigned char>(tmpcvPt1.y, tmpcvPt1.x) = 0; // occupied -> non-free region.
            }

            // dynamic map
        }
    }
    
    nav_msgs::OccupancyGrid occugrid_ground;
    nav_msgs::OccupancyGrid occugrid_obstacle;
    occugrid_ground = mapHandle::cvimg2occumap(occu_ground_map, resolution, original_pt);
    tMap_ground_ = occugrid_ground;
    
    occugrid_obstacle = mapHandle::cvimg2occumap(occu_obstacle_map, resolution, original_pt);
    tMap_obstacle_ =occugrid_obstacle;

    occugrid = mapHandle::cvimg2occumap(occu_ground_map.mul(occu_obstacle_map), resolution, original_pt);
    return occugrid;
}


nav_msgs::OccupancyGrid RoughTerrain_tMap::getOccuObstacle(double resolution){
    nav_msgs::OccupancyGrid occugrid;
    cv::Mat tf_global2pix = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    tf_global2pix.at<float>(0,0) = 1./resolution;
    tf_global2pix.at<float>(1,1) = 1./resolution;
    tf_global2pix.at<float>(2,2) = 1;
    tf_global2pix.at<float>(3,3) = 1;

    double x_min, x_max, y_min, y_max;
    x_min = INFINITY; x_max = 0;
    y_min = INFINITY; y_max = 0;

    int datamin = -40;
    int datastep = 40;
    int datamax = 40;
    cv::Point original_pt;

    // FIND MIN, MAX point of map
    for (int i = 0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_lidar = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_obstacle);
        cv::Mat tmp_ptCloud = cur_pose * cur_lidar;

        double min, max;
        cv::Point minPt, maxPt;

        // Find Min & Max value in X-axis
        cv::minMaxLoc(tmp_ptCloud.row(0), &min, &max, &minPt, &maxPt);
        x_min = (min < x_min)? min : x_min;
        x_max = (max > x_max)? max : x_max;

        // Find Min & Max value in Y-axis
        cv::minMaxLoc(tmp_ptCloud.row(1), &min, &max, &minPt, &maxPt);
        y_min = (min < y_min)? min : y_min;
        y_max = (max > y_max)? max : y_max;
    }

    original_pt = cv::Point(-x_min/resolution, -y_min/resolution);
    tf_global2pix.at<float>(0,3) = -x_min / resolution;
    tf_global2pix.at<float>(1,3) = -y_min / resolution;
    // std::cout << "\033[33;1m" << "x Max: " << x_max << "x Min: " << x_max << "y Max: " << y_max << "y Min: " << y_min << "\033[32;0m"<< std::endl;
    


    ROS_WARN("Drawing the gridMap");
    cv::Mat gridMap = cv::Mat(cv::Size((x_max-x_min)/resolution+1,(y_max-y_min)/resolution+1), CV_8UC1, 120);

    for(int i=0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_lidar = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_obstacle);
        cv::Mat tmp_ptPix = tf_global2pix * cur_pose * cur_lidar;
        cv::Mat tmp_ptPix_c = tf_global2pix * cur_pose;
        cv::Point tmpcvPt_c = cv::Point(tmp_ptPix_c.at<float>(0,3), tmp_ptPix_c.at<float>(1,3));

        if (tmpcvPt_c.x < 0 || tmpcvPt_c.x >= gridMap.size().width || tmpcvPt_c.y < 0 || tmpcvPt_c.y >= gridMap.size().height){
            continue;
        } else {
            for (int k = 0; k<tmp_ptPix.size().width; k++){
                cv::Point tmpcvPt1 = cv::Point(tmp_ptPix.at<float>(0,k), tmp_ptPix.at<float>(1,k));
                if(tmpcvPt1.x < 0|| tmpcvPt1.x >= gridMap.size().width ||tmpcvPt1.y < 0 || tmpcvPt1.y >= gridMap.size().height) continue;

                float dist = sqrt((tmpcvPt_c.x-tmpcvPt1.x)*(tmpcvPt_c.x-tmpcvPt1.x)+(tmpcvPt_c.y-tmpcvPt1.y)*(tmpcvPt_c.y-tmpcvPt1.y));
                int mink = -2;
                int pt_ky = tmpcvPt1.y + 2*mink/dist*(tmpcvPt_c.y-tmpcvPt1.y);
                int pt_kx = tmpcvPt1.x + 2*mink/dist*(tmpcvPt_c.x-tmpcvPt1.x);
                if(pt_kx < 0|| pt_kx >= gridMap.size().width ||pt_ky < 0 || pt_ky >= gridMap.size().height)
                    continue;
                int adddata = 0;

                int tmp_cnt_free =0;
                std::vector<cv::Point> tmp_freecell;
                for(int k = mink; k < (int)dist/2; k++){

                    if(adddata != datamax)
                    {
                    if(k < 0)
                        adddata = datamin;
                    else if(k >= 0 && k <= 1)
                    {
                    }
                    else
                        adddata += datastep;
                    }

                    cv::Point pt_k = cv::Point((tmpcvPt1.x + 2*k/dist*(tmpcvPt_c.x-tmpcvPt1.x)), (tmpcvPt1.y + 2*k/dist*(tmpcvPt_c.y-tmpcvPt1.y)));
                    int tmp_cellData1 = (int)gridMap.at<unsigned char>(pt_k.y, pt_k.x);
                    if(tmp_cellData1 == 0 && adddata > 0)  // obj & free
                    {
                    tmp_cnt_free++;
                    tmp_freecell.push_back(pt_k);
                    }

                    tmp_cellData1 += adddata;
                    if(tmp_cellData1 > 255)
                    tmp_cellData1 = 255;
                    if(tmp_cellData1 < 0)
                    tmp_cellData1 = 0;
                    gridMap.at<unsigned char>(pt_k.y, pt_k.x) = tmp_cellData1;
                }
            }
        }
    }
    
    occugrid = mapHandle::cvimg2occumap( gridMap, resolution, original_pt);
    return occugrid;
}

nav_msgs::OccupancyGrid RoughTerrain_tMap::getOccuGround(double resolution){
    nav_msgs::OccupancyGrid occugrid;
    cv::Mat tf_global2pix = cv::Mat::zeros(cv::Size(4,4), CV_32FC1);
    tf_global2pix.at<float>(0,0) = 1./resolution;
    tf_global2pix.at<float>(1,1) = 1./resolution;
    tf_global2pix.at<float>(2,2) = 1;
    tf_global2pix.at<float>(3,3) = 1;

    double x_min, x_max, y_min, y_max;
    x_min = INFINITY; x_max = 0;
    y_min = INFINITY; y_max = 0;

    int datamin = -40;
    int datastep = 40;
    int datamax = 40;
    cv::Point original_pt;

    // FIND MIN, MAX point of map
    for (int i = 0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_lidar = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_ground);
        cv::Mat tmp_ptCloud = cur_pose * cur_lidar;

        double min, max;
        cv::Point minPt, maxPt;

        // Find Min & Max value in X-axis
        cv::minMaxLoc(tmp_ptCloud.row(0), &min, &max, &minPt, &maxPt);
        x_min = (min < x_min)? min : x_min;
        x_max = (max > x_max)? max : x_max;

        // Find Min & Max value in Y-axis
        cv::minMaxLoc(tmp_ptCloud.row(1), &min, &max, &minPt, &maxPt);
        y_min = (min < y_min)? min : y_min;
        y_max = (max > y_max)? max : y_max;
    }

    original_pt = cv::Point(-x_min/resolution, -y_min/resolution);
    tf_global2pix.at<float>(0,3) = -x_min / resolution;
    tf_global2pix.at<float>(1,3) = -y_min / resolution;
    // std::cout << "\033[33;1m" << "x Max: " << x_max << "x Min: " << x_max << "y Max: " << y_max << "y Min: " << y_min << "\033[32;0m"<< std::endl;
    
    ROS_WARN("Drawing the gridMap");
    cv::Mat gridMap = cv::Mat(cv::Size((x_max-x_min)/resolution+1,(y_max-y_min)/resolution+1), CV_8UC1, 120);

    for(int i=0; i<(int)node_vec_.size(); i++){
        cv::Mat cur_pose = mapHandle::eigen2mat(mapHandle::geoPose2eigen(node_vec_.at(i).lidar_pose));
        cv::Mat cur_lidar = ptHandle::ptCloud2cv(node_vec_.at(i).ptCloud_ground);
        cv::Mat tmp_ptPix = tf_global2pix * cur_pose * cur_lidar;
        cv::Mat tmp_ptPix_c = tf_global2pix * cur_pose;
        cv::Point tmpcvPt_c = cv::Point(tmp_ptPix_c.at<float>(0,3), tmp_ptPix_c.at<float>(1,3));

        if (tmpcvPt_c.x < 0 || tmpcvPt_c.x >= gridMap.size().width || tmpcvPt_c.y < 0 || tmpcvPt_c.y >= gridMap.size().height){
            continue;
        } else {
            for (int k = 0; k<tmp_ptPix.size().width; k++){
                cv::Point tmpcvPt1 = cv::Point(tmp_ptPix.at<float>(0,k), tmp_ptPix.at<float>(1,k));
                if(tmpcvPt1.x < 0|| tmpcvPt1.x >= gridMap.size().width ||tmpcvPt1.y < 0 || tmpcvPt1.y >= gridMap.size().height) continue;

                // float dist = sqrt((tmpcvPt_c.x-tmpcvPt1.x)*(tmpcvPt_c.x-tmpcvPt1.x)+(tmpcvPt_c.y-tmpcvPt1.y)*(tmpcvPt_c.y-tmpcvPt1.y));
                // int mink = -2;
                // int pt_ky = tmpcvPt1.y + 2*mink/dist*(tmpcvPt_c.y-tmpcvPt1.y);
                // int pt_kx = tmpcvPt1.x + 2*mink/dist*(tmpcvPt_c.x-tmpcvPt1.x);
                // if(pt_kx < 0|| pt_kx >= gridMap.size().width ||pt_ky < 0 || pt_ky >= gridMap.size().height)
                //     continue;
                gridMap.at<unsigned char>(tmpcvPt1.y, tmpcvPt1.x) = 255;
            }
        }
    }
    
    occugrid = mapHandle::cvimg2occumap( gridMap, resolution, original_pt);
    return occugrid;
}

#endif