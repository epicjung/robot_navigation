#include "segmatch/segmented_cloud.hpp"

namespace segmatch{

void Segment::calculateCentroid()
{
  std::feclearexcept(FE_ALL_EXCEPT);
  // Find the mean position of a segment.
  double x_mean = 0.0;
  double y_mean = 0.0;
  double z_mean = 0.0;
  const size_t n_points = point_cloud.points.size();
  for (size_t i = 0u; i < n_points; ++i) {
    x_mean += point_cloud.points[i].x / n_points;
    y_mean += point_cloud.points[i].y / n_points;
    z_mean += point_cloud.points[i].z / n_points;
  }

  centroid = PointXYZ(x_mean, y_mean, z_mean);

  // Check that there were no overflows, underflows, or invalid float operations.
  if (std::fetestexcept(FE_OVERFLOW)) {
    LOG(ERROR) << "Overflow error in centroid computation.";
  } else if (std::fetestexcept(FE_UNDERFLOW)) {
    LOG(ERROR) << "Underflow error in centroid computation.";
  } else if (std::fetestexcept(FE_INVALID)) {
    LOG(ERROR) << "Invalid Flag error in centroid computation.";
  } else if (std::fetestexcept(FE_DIVBYZERO)) {
    LOG(ERROR) << "Divide by zero error in centroid computation.";
  }
}

Id SegmentedCloud::getNextId(const Id& begin_counting_from_this_id)
{
    static Id current_id = 0;
    if (begin_counting_from_this_id == 0)
    {
        return ++current_id;
    }    
    else
    {
        current_id = begin_counting_from_this_id;
        return 0;
    }
}

void SegmentedCloud::addValidSegments(const std::vector<pcl::PointIndices>& clusters_to_add,
                    const pcl::PointCloud<PointType>::Ptr reference_cloud)
{
    for (size_t i = 0u; i < clusters_to_add.size(); ++i) 
    {
        std::vector<int> indices = clusters_to_add[i].indices;

        // Create the cluster.
        
        Segment cluster;

        // Assign the cluster an id.
        cluster.segment_id = getNextId();

        // Copy points into cluster.
        for (size_t j = 0u; j < indices.size(); ++j) {
            const size_t index = indices[j];
            CHECK_LT(index, reference_cloud->points.size()) <<
                "Indice is larger than the reference cloud size when adding clusters. " <<
                "Check that the given reference cloud corresponds to the cluster indices.";
            PointType reference_point = reference_cloud->points[index];
            cluster.point_cloud.push_back(reference_point);
        }
        
        if (cluster.segment_id >= 16777216) {
            LOG_IF(ERROR, !g_too_many_segments_to_store_ids_in_intensity) <<
                "Segment Id being passed to point intensity is larger than float values " <<
                "allow. from this point on, intensity will no longer be usable to store segment ids";
            g_too_many_segments_to_store_ids_in_intensity = true;
        }
        for (size_t j = 0u; j < cluster.point_cloud.size(); ++j) {
            cluster.point_cloud.points[j].intensity = cluster.segment_id;
        }      

        cluster.calculateCentroid();
        valid_segments_.insert(std::make_pair(cluster.segment_id, cluster));      
    }
}

size_t SegmentedCloud::getNumberOfValidSegments() const {
    return valid_segments_.size();
}

void SegmentedCloud::segmentedCloudToPcl(pcl::PointCloud<PointType>::Ptr cloud_out)
{
    std::vector<int> permuted_indexes;
    pcl::PointCloud<PointType> cloud;
    for (unsigned int i = 0u; i < getNumberOfValidSegments(); ++i)
    {
        permuted_indexes.push_back(i);
    }
    std::random_shuffle(permuted_indexes.begin(), permuted_indexes.end());
    unsigned int i = 0u;
    for (std::unordered_map<Id, Segment>::const_iterator it = begin(); it != end(); ++it)
    {
        pcl::PointCloud<PointType> segmented_cloud = it->second.point_cloud;
        for (size_t j = 0u; j < segmented_cloud.size(); ++j)
        {
            segmented_cloud.points[j].intensity = permuted_indexes[i];
        }
        cloud += segmented_cloud;
        ++i; 
    }
    cloud.width = 1;
    cloud.height = cloud.points.size();
    *cloud_out = cloud;
}


void SegmentedCloud::clear()
{
    valid_segments_.clear();
}                    
}