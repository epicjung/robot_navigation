#ifndef SEGMATCH_SEGMENTED_CLOUD_HPP_
#define SEGMATCH_SEGMENTED_CLOUD_HPP_
#include <cfenv>
#include <unordered_map>
#include <vector>
#include "segmatch/common.hpp"
#include "segmatch/features.hpp"

extern bool g_too_many_segments_to_store_ids_in_intensity(false);

namespace segmatch  {

struct Segment 
{
    Segment() {}
    bool empty() {return point_cloud.empty();}
    void clear()
    {
        segment_id = kNoId;
        point_cloud.clear();
        features.clear();
    }
    void calculateCentroid();

    Id segment_id = kNoId;
    unsigned int track_id;
    segmatch::Features features;
    pcl::PointCloud<PointType> point_cloud;
    PointXYZ centroid;
};

class SegmentedCloud
{
    private:
        // std::unique_ptr<Descriptors> descriptors_;
        // std::unique_ptr<OpenCvRandomForest> classifier_;
        std::unordered_map<Id, Segment> valid_segments_;
    public: 
        SegmentedCloud(){};
        
        Id getNextId(const Id& begin_counting_from_this_id = 0);

        void addSegments(const std::vector<pcl::PointIndices>& clusters_to_add,
                         const pcl::PointCloud<PointType>::Ptr reference_cloud);

        void clear();                
};
}
#endif