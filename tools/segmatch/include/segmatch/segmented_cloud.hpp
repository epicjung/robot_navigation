#ifndef SEGMATCH_SEGMENTED_CLOUD_HPP_
#define SEGMATCH_SEGMENTED_CLOUD_HPP_
#include <cfenv>
#include <unordered_map>
#include <vector>
#include "segmatch/common.hpp"
#include "segmatch/features.hpp"


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
    Features features;
    pcl::PointCloud<PointType> point_cloud;
    PointXYZ centroid;
};

class SegmentedCloud
{
    private:
        std::unordered_map<Id, Segment> valid_segments_;

    public: 
        bool g_too_many_segments_to_store_ids_in_intensity;

        SegmentedCloud(){};
        
        Id getNextId(const Id& begin_counting_from_this_id = 0);

        void addValidSegments(const std::vector<pcl::PointIndices>& clusters_to_add,
                         const pcl::PointCloud<PointType>::Ptr reference_cloud);
        size_t getNumberOfValidSegments() const;
        bool empty() const {return getNumberOfValidSegments() == 0;}
        void clear(); 

        std::unordered_map<Id, Segment>::const_iterator begin() const {
            return valid_segments_.begin();
        }

        std::unordered_map<Id, Segment>::const_iterator end() const {
            return valid_segments_.end();
        }

        std::unordered_map<Id, Segment>::iterator begin() {
            return valid_segments_.begin();
        }

        std::unordered_map<Id, Segment>::iterator end() {
            return valid_segments_.end();
        }

        void segmentedCloudToPcl(pcl::PointCloud<PointType>::Ptr cloud_out); 
};
}
#endif