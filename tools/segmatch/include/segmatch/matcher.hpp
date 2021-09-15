#ifndef MATCHER_HPP_
#define MATCHER_HPP_

#include <vector>
#include <string>
#include <pcl/segmentation/extract_clusters.h>

#include "segmatch/common.hpp"
#include "segmatch/features.hpp"
#include "segmatch/opencv_random_forest.hpp"
#include "segmatch/segmented_cloud.hpp"

namespace segmatch {

class SegmentMatcher
{
    private:
        pcl::search::KdTree<PointType>::Ptr kd_tree_;
        pcl::EuclideanClusterExtraction<PointType> cluster_extractor_;
        SegmentedCloud clustered_target_cloud_;

    public: 

        SegmentMatcher();
        ~SegmentMatcher();
    
        void init();
        
        void processSourceCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void processTargetCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void loadTargetCloud(pcl::PointCloud<PointType>::Ptr pointcloud);

        void processCloud(const pcl::PointCloud<PointType>::Ptr target_cloud, SegmentedCloud* segmented_cloud);
};

}
#endif