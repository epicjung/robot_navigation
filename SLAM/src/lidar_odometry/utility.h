#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/impl/io.hpp>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <unordered_map>

#include "../visual_odometry/visual_estimator/utility/tic_toc.h"
#include "lshaped_fitting.h"

using namespace std;

typedef pcl::PointXYZI PointType;

struct Cluster
{
    int id;
    pcl::PointCloud<PointType> cloud;
    double centroid_x;
    double centroid_y;
    double centroid_z;
    float feature;
    float vel_x;
    float vel_y;
    jsk_recognition_msgs::BoundingBox bbox;

    Cluster() {feature = 0; vel_x = 0; vel_y = 0; bbox.value = -1;}

    void calculateCentroid()
    {
        double mean_x = 0.0;
        double mean_y = 0.0;
        double mean_z = 0.0;
        const size_t n_points = cloud.points.size();
        for (size_t i = 0u; i <n_points; ++i)
        {
            mean_x += cloud.points[i].x / n_points;
            mean_y += cloud.points[i].y / n_points;
            mean_z += cloud.points[i].z / n_points;
        }
        centroid_x = mean_x;
        centroid_y = mean_y;
        centroid_z = mean_z;
    }

    void fitBoundingBox()
    {
        // check min max z
        PointType min_pt, max_pt;
        pcl::getMinMax3D(cloud, min_pt, max_pt);
        float height = max_pt.z - min_pt.z;
        printf("%d------\n", id);
        // printf("height: %f\n", height);
        if (height < 0.5 || height > 2.60)
            return;
        
        
        std::vector<cv::Point2f> hull;
        for (size_t i = 0u; i < cloud.points.size(); ++i)
        {
            hull.push_back(cv::Point2f(cloud.points[i].x, cloud.points[i].y));
        }

        LShapedFIT lshaped;
        cv::RotatedRect rrect = lshaped.FitBox(&hull);
        // std::cout << "Shaped-BBox Message : " << rrect.size.width << " " << rrect.size.height << " " << rrect.angle << std::endl;
        
        // check area
        float area = rrect.size.width * rrect.size.height;
        // printf("area: %f\n", area);
        if (area > 20.0) 
            return;
        
        // check ratio
        float ratio = rrect.size.height > rrect.size.width ? rrect.size.height / rrect.size.width : rrect.size.width / rrect.size.height;
        // printf("ratio: %f\n", ratio);
        if (ratio >= 5.0)
            return;

        float density = cloud.points.size() / (rrect.size.width * rrect.size.height * height);
        // printf("density: %f\n", density);
        if (density < 3.0) 
            return;
        
        std::vector<cv::Point2f> vertices = lshaped.getRectVertex();
        cv::Point3f center;
        center.z = (max_pt.z + min_pt.z) / 2.0;
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            center.x += vertices[i].x / vertices.size();
            center.y += vertices[i].y / vertices.size();
        }
        bbox.pose.position.x = center.x;
        bbox.pose.position.y = center.y;
        bbox.pose.position.z = center.z;
        bbox.dimensions.x = rrect.size.width;
        bbox.dimensions.y = rrect.size.height;
        bbox.dimensions.z = height;
        bbox.value = 0.0;
        printf("Centers: %f;%f;%f \n", center.x, center.y, center.z);
        tf::Quaternion quat = tf::createQuaternionFromYaw(rrect.angle * M_PI / 180.0);
        tf::quaternionTFToMsg(quat, bbox.pose.orientation);
    }
};

enum Type {QUERY, REFERENCE};

struct ClusterMap
{
    pcl::EuclideanClusterExtraction<PointType> cluster_extractor_;
    std::vector<pcl::PointIndices> cluster_indices_;
    pcl::search::KdTree<PointType>::Ptr kd_tree_;
    pcl::PointCloud<PointType>::Ptr cloud_map_;
    std::vector<Cluster> cluster_map_;

    Type type_;
    double max_r_;
    double max_z_;

    Type getType() {return type_;}
    std::vector<Cluster> getMap(){return cluster_map_;}
    std::vector<pcl::PointIndices> getClusterIndices(){return cluster_indices_;}
    void setFeature(int id, float feature)
    {
        for (std::vector<Cluster>::iterator it = cluster_map_.begin(); it != cluster_map_.end(); ++it)
        {
            if (it->id == id)
            {
                if (it->feature == 0.0 || feature < it->feature)
                    it->feature = feature;
            }
        }
    }

    void init(Type type, double max_r, double max_z, double tol, double min_cluster_size, double max_cluster_size)
    {
        type_ = type;
        max_r_ = max_r;
        max_z_ = max_z;
        kd_tree_.reset(new pcl::search::KdTree<PointType>());
        cloud_map_.reset(new pcl::PointCloud<PointType>());
        cluster_extractor_.setClusterTolerance(tol);
        cluster_extractor_.setMinClusterSize(min_cluster_size);
        cluster_extractor_.setMaxClusterSize(max_cluster_size);
        cluster_extractor_.setSearchMethod(kd_tree_);
    }

    void buildMap(pcl::PointCloud<PointType>::Ptr cloud_in, int &start_id)
    {
        TicToc tic_toc;

        *cloud_map_ = *cloud_in;

        cluster_indices_.clear();
        cluster_extractor_.setInputCloud(cloud_map_);
        cluster_extractor_.extract(cluster_indices_);
        printf("Points: %d, # of segments: %d\n", cloud_map_->points.size(), (int)cluster_indices_.size());
        ROS_WARN("Clustering: %f ms\n", tic_toc.toc());
        addClusters(cluster_indices_, start_id);
    }

    void addClusters(std::vector<pcl::PointIndices> clusters, int &start_id)
    {
        TicToc tic_toc;
        for (size_t i = 0u; i < clusters.size(); ++i)
        {
            Cluster cluster;
            cluster.id = (int) start_id++;
            std::vector<int> indices = clusters[i].indices;
            for (size_t j = 0u; j <indices.size(); ++j)
            {
                const size_t index = indices[j];
                PointType point = cloud_map_->points[index];
                cluster.cloud.points.push_back(point);
            }
            cluster.calculateCentroid();
            cluster.fitBoundingBox();
            cluster_map_.push_back(cluster);
        }
        ROS_WARN("fit bounding box: %f ms\n", tic_toc.toc());
    }

    void clear()
    {
        cluster_map_.clear();
    }
};

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string PROJECT_NAME;

    std::string robot_id;

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    int N_SCAN;
    int Horizon_SCAN;
    string timeField;
    int downsampleRate;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool loopClosureEnableFlag;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // euigon
    float nongroundDownsample;
    float maxZ;
    float minZ;
    float maxR;
    float errorThres; 
    double clusteringTolerance;
    int minClusterSize;
    int maxClusterSize;

    ParamServer()
    {
        nh.param<std::string>("/PROJECT_NAME", PROJECT_NAME, "sam");

        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>(PROJECT_NAME + "/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>(PROJECT_NAME + "/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>(PROJECT_NAME + "/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>(PROJECT_NAME + "/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<bool>(PROJECT_NAME + "/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>(PROJECT_NAME + "/useGpsElevation", useGpsElevation, false);
        nh.param<float>(PROJECT_NAME + "/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>(PROJECT_NAME + "/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>(PROJECT_NAME + "/savePCD", savePCD, false);
        nh.param<std::string>(PROJECT_NAME + "/savePCDDirectory", savePCDDirectory, "/tmp/loam/");

        nh.param<int>(PROJECT_NAME + "/N_SCAN", N_SCAN, 16);
        nh.param<int>(PROJECT_NAME + "/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<std::string>(PROJECT_NAME + "/timeField", timeField, "time");
        nh.param<int>(PROJECT_NAME + "/downsampleRate", downsampleRate, 1);

        nh.param<float>(PROJECT_NAME + "/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>(PROJECT_NAME + "/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>(PROJECT_NAME + "/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>(PROJECT_NAME + "/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>(PROJECT_NAME + "/imuGravity", imuGravity, 9.80511);
        nh.param<vector<double>>(PROJECT_NAME+ "/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>(PROJECT_NAME+ "/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>(PROJECT_NAME+ "/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>(PROJECT_NAME + "/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>(PROJECT_NAME + "/surfThreshold", surfThreshold, 0.1);
        nh.param<int>(PROJECT_NAME + "/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>(PROJECT_NAME + "/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>(PROJECT_NAME + "/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>(PROJECT_NAME + "/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>(PROJECT_NAME + "/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>(PROJECT_NAME + "/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>(PROJECT_NAME + "/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>(PROJECT_NAME + "/numberOfCores", numberOfCores, 2);
        nh.param<double>(PROJECT_NAME + "/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>(PROJECT_NAME + "/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>(PROJECT_NAME + "/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>(PROJECT_NAME + "/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>(PROJECT_NAME + "/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>(PROJECT_NAME + "/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<int>(PROJECT_NAME + "/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>(PROJECT_NAME + "/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>(PROJECT_NAME + "/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>(PROJECT_NAME + "/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        // euigon
        nh.param<float>(PROJECT_NAME + "/nongroundDownsample", nongroundDownsample, 1e3);
        nh.param<float>(PROJECT_NAME + "/maxZ", maxZ, 10.0);
        nh.param<float>(PROJECT_NAME + "/minZ", minZ, 10.0);
        nh.param<float>(PROJECT_NAME + "/maxR", maxR, 1.0);
        nh.param<float>(PROJECT_NAME + "/errorThres", errorThres, 1.0);
        nh.param<double>(PROJECT_NAME + "/clusteringTolerance", clusteringTolerance, 10.0);
        nh.param<int>(PROJECT_NAME + "/minClusterSize", minClusterSize, 1.0);
        nh.param<int>(PROJECT_NAME + "/maxClusterSize", maxClusterSize, 1.0);
        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, T thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif