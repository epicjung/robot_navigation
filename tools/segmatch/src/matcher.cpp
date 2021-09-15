#include "segmatch/matcher.hpp"

namespace segmatch{

SegmentMatcher::SegmentMatcher()
{
    // set euclidean clustering 
    kd_tree_.reset(new pcl::search::KdTree<PointType>());
    cluster_extractor_.setClusterTolerance(0.5);
    cluster_extractor_.setMinClusterSize(5);
    cluster_extractor_.setMaxClusterSize(1000);
    cluster_extractor_.setSearchMethod(kd_tree_);
}

SegmentMatcher::~SegmentMatcher()
{
    kd_tree_.reset();
}

void SegmentMatcher::init() 
{

}

void SegmentMatcher::processSourceCloud(pcl::PointCloud<PointType>::Ptr source_cloud)
{

}

void SegmentMatcher::processTargetCloud(pcl::PointCloud<PointType>::Ptr target_cloud)
{
    processCloud(target_cloud, &clustered_target_cloud_);
}

void SegmentMatcher::processCloud(pcl::PointCloud<PointType>::Ptr target_cloud, ClusteredCloud *clustered_cloud)
{
    // Perform clustering
    // TicToc tic_toc;
    clustered_cloud->clear();
    pcl::PointCloud<PointType>::Ptr copied_cloud(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*target_cloud, *copied_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extractor_.setInputCloud(copied_cloud);
    cluster_extractor_.extract(cluster_indices);
    clustered_cloud->addClusters(cluster_indices, copied_cloud);
    // printf("Clustering finished. %d clusters and took %f ms\n", (int)cluster_indices.size(), tic_toc.toc());
}

}