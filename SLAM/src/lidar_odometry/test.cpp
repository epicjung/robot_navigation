#include "utility.h"
#include "lvi_sam/cloud_info.h"
#include "GMM.h"

struct Segment
{
    Segment() {}
    void clear() {
        cloud.clear();
        normals.clear();
    }

    void calculateCentroid()
    {
        double xMean = 0.0;
        double yMean = 0.0;
        double zMean = 0.0;
        double rMean = 0.0;
        double thetaMean = 0.0;
        const size_t nPoints = cloud.points.size();
        for (size_t i = 0u; i < nPoints; ++i)
        {
            xMean += cloud.points[i].x / nPoints;
            yMean += cloud.points[i].y / nPoints;
            zMean += cloud.points[i].z / nPoints;
            rMean += cloud.points[i].normal_x / nPoints;
            thetaMean += cloud.points[i].normal_y / nPoints;
        }
        centroidX = xMean;
        centroidY = yMean;
        centroidZ = zMean;
        meanR = rMean;
        meanTheta = thetaMean;
        meanZ = zMean;
        printf("Segment: %d, meanR: %f, meanTheta: %f, meanZ: %f\n", segmentId, meanR, meanTheta * 180 / M_PI, meanZ);
    }

    int segmentId;
    pcl::PointCloud<PointType2> cloud;
    pcl::PointCloud<pcl::Normal> normals;
    double centroidX;
    double centroidY;
    double centroidZ;
    double meanR;
    double meanTheta;
    double meanZ;
};

// class SegmentedCloud
// {
//     public:

//         int currentId;
//         std::unordered_map<int, Segment> validSegments;

//         SegmentedCloud() 
//         {
//            currentId = 0;
//         };

//         void addValidSegment(const Segment& segment_to_add)
//         {

//         }

//         double xy2theta(const double &x, const double &y)
//         {
//             if (y >= 0)
//                 return atan2(y,x);
//             else
//                 return 2*M_PI + atan2(y,x);
//         }

//         double xy2radius(const double &x, const double &y)
//         {
//             return sqrt(x*x+y*y);
//         }

//         void calculateCentroid(Segment &segment, pcl::PointCloud<PointType> cloud)
//         {
//             double xMean = 0.0;
//             double yMean = 0.0;
//             double zMean = 0.0;
//             const size_t nPoints = cloud.points.size();
//             for (size_t i = 0u; i < nPoints; ++i)
//             {
//                 xMean += cloud.points[i].x / nPoints;
//                 yMean += cloud.points[i].y / nPoints;
//                 zMean += cloud.points[i].z / nPoints;
//             }
//             segment.centroidX = xMean;
//             segment.centroidY = yMean;
//             segment.centroidZ = zMean;
//             segment.meanR = xy2radius(xMean, yMean);
//             segment.meanTheta = xy2theta(xMean, yMean);
//         }

//         void addValidSegments(const std::vector<pcl::PointIndices> segments, const pcl::PointCloud<PointType>::Ptr refCloud)
//         {
//             for (size_t i = 0u; i < segments.size(); ++i)
//             {
//                 Segment segment;
//                 segment.segmentId = currentId++;
//                 std::vector<int> indices = segments[i].indices;
                
//                 for (size_t j = 0u; j < indices.size(); ++j)
//                 {
//                     const size_t index = indices[j];
//                     PointType refPoint = refCloud->points[index];
//                     PointType point;
//                     point.x = refPoint.x;
//                     point.y = refPoint.y;
//                     point.z = refPoint.z;
//                     segment.cloud.push_back(point);
//                 }

//                 for (size_t j = 0u; j < segment.cloud.size(); ++j)
//                 {
//                     segment.cloud.points[j].intensity = segment.segmentId;
//                 }
//                 calculateCentroid(segment, segment.cloud);
//                 validSegments.insert(std::make_pair(segment.segmentId, segment));
//             }
//         }
// };

struct RayMap
{
    vector<pcl::PointCloud<PointType>::Ptr> sectors;
    vector<boost::shared_ptr<Gaussian_Mixture_Model>> gaussianMixtureModels;
    vector<Segment> validSegments;
    
    int numSectors_;
    int segIdCurr_;
    
    float xy2theta(const float &x, const float &y)
    {
        if (y >= 0)
            return atan2(y,x);
        else
            return 2*M_PI + atan2(y,x);
    }

    float xy2radius(const float &x, const float &y)
    {
        return sqrt(x*x+y*y);
    }

    void setGaussianMixtureModel(int sectorId, vector<Segment> segmentsToAdd)
    {
        TicToc tic_toc;
        int dataDim = 3;
        int numGauss = (int) segmentsToAdd.size();
        int numIter = 30;
        // create Gaussian Mixture Model 
        pcl::PointCloud<PointType2>::Ptr sectorCloud(new pcl::PointCloud<PointType2>);
        gaussianMixtureModels.at(sectorId)->SetParameter("diagonal", dataDim, numGauss); 
        
        for (int i = 0; i < numGauss; ++i)
        {
            // set means
            gaussianMixtureModels.at(sectorId)->mean[i][0] = segmentsToAdd[i].meanR;
            gaussianMixtureModels.at(sectorId)->mean[i][1] = segmentsToAdd[i].meanTheta;
            gaussianMixtureModels.at(sectorId)->mean[i][2] = segmentsToAdd[i].meanZ;
            gaussianMixtureModels.at(sectorId)->weight[i] = 1.0 / numGauss;           
            *sectorCloud += segmentsToAdd[i].cloud;
            
            // set covariances
            for (int j = 0; j < dataDim; j++){
                gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][j] = 1;
            }
        }
        
        // set data
        int numData = (int) sectorCloud->points.size();
        double **data = new double*[numData];
        for (int i = 0; i < numData; ++i)
        {
            data[i] = new double[dataDim];
            data[i][0] = (double)sectorCloud->points[i].normal_x;
            data[i][1] = (double)sectorCloud->points[i].normal_y;
            data[i][2] = (double)sectorCloud->points[i].normal_z;
        }

        printf("sector Id: %d, numGauss: %d, numData:%d\n", sectorId, numGauss, numData);

        printf("step	log_likelihood\n");
        for (int i = 0; i < numIter; i++)
        {
            double log_likelihood = gaussianMixtureModels.at(sectorId)->Expectaion_Maximization(numData, data);
            if ((i + 1) % 10 == 0) printf("%d	%lf\n", i + 1, log_likelihood);
        }
        printf("Guassian Model Time: %f\n", tic_toc.toc());

        // test
        TicToc inference;
        for (int i = 0; i < numGauss; i++){
            for (int j = 0; j < dataDim; j++){
                printf("%lf ", gaussianMixtureModels.at(sectorId)->mean[i][j]);
            }
            printf("\n");
	    }

        double likelihoodSum = 0.0;
        for (int i = 0; i < numData; ++i)
        {
            likelihoodSum += gaussianMixtureModels.at(sectorId)->Calculate_Likelihood(data[i]);
        }
        printf("Likelihood: %f\n", likelihoodSum);
        printf("Inference Time: %f\n", inference.toc());
    }

    void addSegments(int sectorId, std::vector<pcl::PointIndices> segments, pcl::PointCloud<PointType>::Ptr refCloud)
    {
        vector<Segment> segmentsToAdd;
        for (size_t i = 0u; i < segments.size(); ++i)
        {
            Segment segment;
            segment.segmentId = segIdCurr_++;
            std::vector<int> indices = segments[i].indices;
            for (size_t j = 0u; j < indices.size(); ++j)
            {
                const size_t index = indices[j];
                PointType refPoint = refCloud->points[index];
                PointType2 point;
                point.x = refPoint.x;
                point.y = refPoint.y;
                point.z = refPoint.z;
                point.normal_x = xy2radius(refPoint.x, refPoint.y); // radial distance
                point.normal_y = xy2theta(refPoint.x, refPoint.y); // azimuth angle
                point.normal_z = point.z;
                point.label = segment.segmentId;
                segment.cloud.points.push_back(point);
            }
            segment.calculateCentroid();
            segmentsToAdd.push_back(segment);
            validSegments.push_back(segment);
        }
        setGaussianMixtureModel(sectorId, segmentsToAdd);
    }

    void init(int numSectors)
    {
        segIdCurr_ = 0;
        numSectors_ = numSectors;
        sectors.reserve(numSectors);
        for (int i = 0; i < (int)numSectors; ++i)
        {
            pcl::PointCloud<PointType>::Ptr sectorCloud(new pcl::PointCloud<PointType>);
            sectors.emplace_back(sectorCloud);
        }
    }

    void clear()
    {
        segIdCurr_ = 0;
        validSegments.clear();
        gaussianMixtureModels.clear();
        gaussianMixtureModels.reserve(numSectors_);
        for (int i = 0; i < (int)numSectors_; ++i)
        {
            if (!sectors[i]->points.empty())
                sectors[i]->points.clear();
            boost::shared_ptr<Gaussian_Mixture_Model> GMM;
            GMM.reset(new Gaussian_Mixture_Model());
            gaussianMixtureModels.emplace_back(GMM);
        }
    }
};



class Test : public ParamServer
{

public:

    ros::Subscriber subLaserCloud;
    ros::Subscriber subCloudInfo;
    ros::Subscriber subNonGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubCentroidCloud;
    lvi_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;
    deque<lvi_sam::cloud_info> cloudInfoQueue;
    pcl::EuclideanClusterExtraction<PointType> clusterExtractor_;
    pcl::search::KdTree<PointType>::Ptr kdTree_;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    RayMap rayMap;
    double maxR;
    int numSectors;

    Test()
    {
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test", 5, &Test::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        // subCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, &Test::cloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subNonGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/GSeg/nonground_ptCloud", 5, &Test::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/segmented_cloud", 1);
        pubCentroidCloud  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/centroids", 1);
        initializationValue();
    }

    ~Test()
    {
        kdTree_.reset();
    }

    void initializationValue()
    {
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        kdTree_.reset(new pcl::search::KdTree<PointType>());
        clusterExtractor_.setClusterTolerance(0.5);
        clusterExtractor_.setMinClusterSize(5);
        clusterExtractor_.setMaxClusterSize(1000);
        clusterExtractor_.setSearchMethod(kdTree_);
        numSectors = 36;
        maxR = 100.0;
        rayMap.init(numSectors);
        printf("RayMap initialized\n");
    }

    double xy2theta(const double &x, const double &y)
    {
        if (y >= 0)
            return atan2(y,x);
        else
            return 2*M_PI + atan2(y,x);
    }

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
    {
        extractedCloud->clear();
        rayMap.clear();
        pcl::fromROSMsg(*msgIn, *extractedCloud);

        double thetaIncre = 360.0 / numSectors * (M_PI / 180.0); // [rad]
        for (auto &pt : extractedCloud->points)
        {
            double radius = rayMap.xy2radius(pt.x, pt.y);
            if (radius <= maxR && pt.z <= 1.5)
            {
                double theta = rayMap.xy2theta(pt.x, pt.y);
                int sectorId = min(static_cast<int>(theta / thetaIncre), numSectors - 1);
                // pt.intensity = static_cast<float>(sectorId + 1); 
                rayMap.sectors.at(sectorId)->points.push_back(pt);
            }
            else
            {
                // pt.intensity = 0.0;
            }
        }

        pcl::PointCloud<PointType2>::Ptr outPcl (new pcl::PointCloud<PointType2>);
        pcl::PointCloud<PointType>::Ptr centroids (new pcl::PointCloud<PointType>);
        // SegmentedCloud *segmentedCloud = new SegmentedCloud();

        for (int i = 0; i < (int)rayMap.sectors.size(); ++i)
        {
            std::vector<pcl::PointIndices> clusterIndices;
            clusterExtractor_.setInputCloud(rayMap.sectors[i]);
            clusterExtractor_.extract(clusterIndices);
            printf("Sector %d: seg size: %d point size: %d\n", i, (int)clusterIndices.size(), (int)rayMap.sectors[i]->points.size());
            rayMap.addSegments(i, clusterIndices, rayMap.sectors[i]);
            // segmentedCloud->addValidSegments(clusterIndices, rayMap.sectors[i]);
        }
        for (auto seg : rayMap.validSegments)
        {
            *outPcl += seg.cloud;
            PointType centroid;
            centroid.x = seg.centroidX;
            centroid.y = seg.centroidY;
            centroid.z = seg.centroidZ;
            centroid.intensity = 100.0;
            centroids->points.emplace_back(centroid);
        }
        printf("outPCL size: %d\n", (int)outPcl->points.size());
        sensor_msgs::PointCloud2 outCloud;
        sensor_msgs::PointCloud2 centroidCloud;
        pcl::toROSMsg(*outPcl, outCloud);
        pcl::toROSMsg(*centroids, centroidCloud);
        outCloud.header.stamp = cloudHeader.stamp;
        outCloud.header.frame_id = "vins_body_ros";
        centroidCloud.header = outCloud.header;
        pubSegmentedCloud.publish(outCloud);
        pubCentroidCloud.publish(centroidCloud);
    }

    void cloudInfoHandler(const lvi_sam::cloud_info::ConstPtr &msgIn)
    {
        // cloudInfoQueue.push_back(*msgIn);
        // extractedCloud->clear();
        // rayMap.clear();
        // pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);

        // double thetaIncre = 360.0 / numSectors * (M_PI / 180.0); // [rad]
        // for (auto &pt : extractedCloud->points)
        // {
        //     double radius = sqrt(pt.x*pt.x + pt.y + pt.y);
        //     if (radius <= maxR)
        //     {
        //         double theta = xy2theta(pt.x, pt.y);
        //         int sectorId = min(static_cast<int>(theta / thetaIncre), numSectors - 1);
        //         // pt.intensity = static_cast<float>(sectorId + 1); 
        //         sector.at(sectorId)->points.push_back(pt);
        //     }
        //     else
        //     {
        //         // pt.intensity = 0.0;
        //     }
        // }

        // pcl::PointCloud<PointType>::Ptr outPcl (new pcl::PointCloud<PointType>);
        // SegmentedCloud *segmentedCloud = new SegmentedCloud();

        // for (int i = 0; i < (int)sector.size(); ++i)
        // {
        //     std::vector<pcl::PointIndices> clusterIndices;
        //     clusterExtractor_.setInputCloud(sector[i]);
        //     clusterExtractor_.extract(clusterIndices);
        //     segmentedCloud->addValidSegments(clusterIndices, sector[i]);
        //     printf("Sector %d: seg size: %d point size: %d\n", i, (int)clusterIndices.size(), (int)sector[i]->points.size());
        // }
        // for (auto seg : segmentedCloud->validSegments)
        // {
        //     *outPcl += seg.second.cloud;
        // }
        // printf("outPCL size: %d\n", (int)outPcl->points.size());
        // sensor_msgs::PointCloud2 outCloud;
        // pcl::toROSMsg(*outPcl, outCloud);
        // outCloud.header.stamp = cloudHeader.stamp;
        // outCloud.header.frame_id = "vins_body_ros";
        // pubSegmentedCloud.publish(outCloud);
    }

    // void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& msgIn)
    // {
    //     cloudHeader = msgIn->header; // new cloud header
    //     pcl::fromROSMsg(*msgIn, *extractedCloud); // new cloud for extraction

    //     SegmentedCloud *segmentedCloud = new SegmentedCloud();

    //     std::vector<pcl::PointIndices> clusterIndices;
    //     clusterExtractor_.setInputCloud(extractedCloud);
    //     clusterExtractor_.extract(clusterIndices);
        
    //     printf("clusterIndices: %d\n", (int)clusterIndices.size());
    //     // pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    //     // pcl::copyPointCloud<PointType, pcl::PointNormal>(*extractedCloud, *cloudWithNormals);
    //     printf("Copied successfully\n");
    //     // segmentedCloud_->addValidSegments(clusterIndices, *cloudWithNormals);
    //     segmentedCloud->addValidSegments(clusterIndices, extractedCloud);
    //     printf("Segmented successfully\n");

    //     pcl::PointCloud<PointType>::Ptr outPcl (new pcl::PointCloud<PointType>);

    //     for (auto seg : segmentedCloud->validSegments)
    //     {
    //         *outPcl += seg.second.cloud;
    //     }

    //     printf("outPcl size: %d\n", (int)outPcl->points.size());
        
    //     sensor_msgs::PointCloud2 outCloud;
    //     pcl::toROSMsg(*outPcl, outCloud);
    //     outCloud.header.stamp = cloudHeader.stamp;
    //     outCloud.header.frame_id = "vins_body_ros";
    //     pubSegmentedCloud.publish(outCloud);
        
    // }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");

    Test test;
   
    ros::spin();

    return 0;
}