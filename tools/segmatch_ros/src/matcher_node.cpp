#include "segmatch_ros/matcher.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segment_matcher");

    ros::NodeHandle nh;

    SegmentMatcher matcher;

    matcher.init(nh);

    ros::spin();

    return 0;
}