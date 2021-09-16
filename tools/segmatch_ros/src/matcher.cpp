#include "segmatch_ros/matcher.hpp"

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

void SegmentMatcher::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
{
    pcl::PointCloud<PointType>::Ptr cloudIn(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msgIn, *cloudIn);
    processTargetCloud(cloudIn);
    publishTargetRepresentation(msgIn->header.stamp, "base_link");
}

void SegmentMatcher::init(ros::NodeHandle& nh) 
{
    sub_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/GSeg/nonground_ptCloud", 5, &SegmentMatcher::cloudHandler, this, ros::TransportHints().tcpNoDelay());
    pub_segmented_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/SegmentMacther/segmented_cloud", 5);
}

void SegmentMatcher::processSourceCloud(pcl::PointCloud<PointType>::Ptr source_cloud)
{

}

void SegmentMatcher::processTargetCloud(pcl::PointCloud<PointType>::Ptr target_cloud)
{
    processCloud(target_cloud, &clustered_target_cloud_);
    classifier_->setTarget(clustered_target_cloud_);
    target_cloud_loaded_ = true;
}

void SegmentMatcher::processCloud(pcl::PointCloud<PointType>::Ptr target_cloud, SegmentedCloud *clustered_cloud)
{
    // Perform clustering
    // TicToc tic_toc;
    clustered_cloud->clear();
    pcl::PointCloud<PointType>::Ptr copied_cloud(new pcl::PointCloud<PointType>);
    pcl::copyPointCloud(*target_cloud, *copied_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_extractor_.setInputCloud(copied_cloud);
    cluster_extractor_.extract(cluster_indices);
    clustered_cloud->addValidSegments(cluster_indices, copied_cloud);
    // printf("Clustering finished. %d clusters and took %f ms\n", (int)cluster_indices.size(), tic_toc.toc());

    // LOG(INFO) << "Removing too near segments from source map.";

    // LOG(INFO) << "Extract descriptors";
    std::vector<double> segmentation_timings;
    // descriptors_->describe(clustered_cloud, &segmentation_timings);
}

void SegmentMatcher::publishTargetRepresentation(ros::Time time, std::string frame)
{
  	// Convert segmented cloud to pcl cloud
	pcl::PointCloud<PointType>::Ptr segmented_cloud(new pcl::PointCloud<PointType>);
	clustered_target_cloud_.segmentedCloudToPcl(segmented_cloud);

	// Convert pcl cloud to pointcloud2 message
    sensor_msgs::PointCloud2 cloud_out; 
	pcl::toROSMsg(*segmented_cloud, cloud_out);
	cloud_out.header.stamp = time;
	cloud_out.header.frame_id = frame;
    pub_segmented_cloud_.publish(cloud_out);
}
