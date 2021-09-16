#ifndef MATCHER_HPP_
#define MATCHER_HPP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <glog/logging.h>
#include <pcl/segmentation/extract_clusters.h>

#include "segmatch/common.hpp"
#include "segmatch/features.hpp"
#include "segmatch/opencv_random_forest.hpp"
#include "segmatch/segmented_cloud.hpp"
// #include "segmatch/descriptors/descriptors.hpp"

using namespace segmatch;

class SegmentMatcher
{
    private:
        pcl::search::KdTree<PointType>::Ptr kd_tree_;
        pcl::EuclideanClusterExtraction<PointType> cluster_extractor_;
        segmatch::SegmentedCloud clustered_target_cloud_;
        // std::unique_ptr<segmatch::Descriptors> descriptors_;
        std::unique_ptr<segmatch::OpenCvRandomForest> classifier_;
        bool target_cloud_loaded_ = false;

        // publishers & subscribers
        ros::Subscriber sub_cloud_;
        ros::Publisher  pub_segmented_cloud_;
        
    public: 

        SegmentMatcher();
        ~SegmentMatcher();

        void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msgIn);
    
        void init(ros::NodeHandle &nh);
        
        void processSourceCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void processTargetCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void publishTargetRepresentation(ros::Time time, std::string frame);

        void loadTargetCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void processCloud(const pcl::PointCloud<PointType>::Ptr target_cloud, SegmentedCloud* segmented_cloud);
};

#endif