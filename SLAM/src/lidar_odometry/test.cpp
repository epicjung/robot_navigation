#include "utility.h"
#include "lvi_sam/cloud_info.h"
#include "GMM.h"

int LIDAR_SKIP = 3;


struct Segment
{
    Segment() {
        staticProb = 0.0;
    }
    void clear() {
        cloud.clear();
        normals.clear();
        staticProb = 0.0;
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
        // printf("Segment: %d, meanR: %f, meanTheta: %f, meanZ: %f\n", segmentId, meanR, meanTheta * 180 / M_PI, meanZ);
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
    double staticProb;
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

enum Type {LOCAL, GLOBAL};

struct RayMap
{
    vector<pcl::PointCloud<PointType>::Ptr> sectors;
    pcl::EuclideanClusterExtraction<PointType> clusterExtractor_;
    pcl::search::KdTree<PointType>::Ptr kdTree_;
    vector<boost::shared_ptr<Gaussian_Mixture_Model>> gaussianMixtureModels;
    unordered_map<int, vector<Segment>> validSegments;

    int numSectors_;
    Type type_;
    double maxR_;
    double maxZ_;
    double thetaRes_;
    int segIdCurr_;
    double FOV_;
    double radialStd = 0.05;
    
    float xy2theta(const float &x, const float &y)
    {
        return atan2(y, x);
    }

    float xy2radius(const float &x, const float &y)
    {
        return sqrt(x*x+y*y);
    }

    Type getType() {return type_;}

    visualization_msgs::MarkerArray getRayMap()
    {
        visualization_msgs::MarkerArray raymap;
        visualization_msgs::Marker lines;
        lines.header.frame_id = "base_link";
        lines.scale.x = 0.05;
        lines.color.b = 1.0;
        lines.color.a = 1.0;
        lines.type = 5; //LINE_LIST
        lines.id = 100;
        lines.action = 0;
        for (int i = 0; i < numSectors_; ++i)
        {
            // get sector
            geometry_msgs::Point origin;
            origin.x = 0.0;
            origin.y = 0.0;
            origin.z = 0.0;
            geometry_msgs::Point endpoint;
            endpoint.x = 20.0 * cos(thetaRes_ * i - FOV_ / 2.0);
            endpoint.y = 20.0 * sin(thetaRes_ * i - FOV_ / 2.0);
            endpoint.z = 0.0; 
            lines.points.push_back(origin);
            lines.points.push_back(endpoint);
            
            // get statistic
            
            visualization_msgs::Marker text;
            text.header.frame_id = "base_link";
            text.scale.z = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.action = 0;
            text.type = 9; // TEXT_VIEW_FACING
            text.id = i;
            text.text = to_string(i);
            text.pose.position.x = endpoint.x;
            text.pose.position.y = endpoint.y;
            text.pose.orientation.w = 1.0;
            raymap.markers.push_back(text);
        }
        raymap.markers.push_back(lines);
        return raymap;
    }

    void setRayMap(pcl::PointCloud<PointType>::Ptr laserCloudIn)
    {
        // 1. build ray map for current scan
        printf("Total size: %d\n", (int)laserCloudIn->points.size());
        for (auto &pt : laserCloudIn->points)
        {
            double radius = xy2radius(pt.x, pt.y);
            double theta = xy2theta(pt.x, pt.y);
            if (radius <= maxR_ && pt.z <= maxZ_ && abs(theta) <= FOV_ / 2.0)
            {
                int sectorId = min(static_cast<int>((theta + FOV_ / 2.0) / thetaRes_), numSectors_ - 1);
                // pt.intensity = static_cast<float>(sectorId + 1); 
                sectors.at(sectorId)->points.push_back(pt);
            }
            else
            {
                // pt.intensity = 0.0;
            }
        }

        // 2. Gaussian mixture odeling of the ray map
        for (int i = 0; i < (int)sectors.size(); ++i)
        {
            std::vector<pcl::PointIndices> clusterIndices;
            clusterExtractor_.setInputCloud(sectors[i]);
            clusterExtractor_.extract(clusterIndices);
            printf("Sector %d: seg size: %d point size: %d\n", i, (int)clusterIndices.size(), (int)sectors[i]->points.size());
            addSegments(i, clusterIndices, sectors[i]);
        }
    }

    void setGaussianModel(int sectorId, vector<Segment> segmentsToAdd)
    {
        TicToc tic_toc;
        int dataDim = 3;
        int numGauss = (int) segmentsToAdd.size();
        gaussianMixtureModels.at(sectorId)->SetParameter("diagonal", dataDim, numGauss); 
        pcl::PointCloud<PointType2>::Ptr segmentCloud(new pcl::PointCloud<PointType2>);
        for (int i = 0; i < numGauss; ++i)
        {
            // set means
            gaussianMixtureModels.at(sectorId)->mean[i][0] = segmentsToAdd[i].meanR;
            gaussianMixtureModels.at(sectorId)->mean[i][1] = segmentsToAdd[i].meanTheta;
            // gaussianMixtureModels.at(sectorId)->mean[i][2] = segmentsToAdd[i].meanZ;
            gaussianMixtureModels.at(sectorId)->weight[i] = 1.0 / numGauss;

            for (int k = 0; k < dataDim; ++k)
            {
                gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][k] = 0.0;           
            }

            for (auto &pt : segmentsToAdd[i].cloud)
            {
                // set covariances
                gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][0] += ((pt.normal_x - gaussianMixtureModels.at(sectorId)->mean[i][0])*(pt.normal_x - gaussianMixtureModels.at(sectorId)->mean[i][0])) + radialStd * radialStd;
                gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][1] += ((pt.normal_y - gaussianMixtureModels.at(sectorId)->mean[i][1])*(pt.normal_y - gaussianMixtureModels.at(sectorId)->mean[i][1]));
                gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][2] += ((pt.normal_z - gaussianMixtureModels.at(sectorId)->mean[i][2])*(pt.normal_z - gaussianMixtureModels.at(sectorId)->mean[i][2]));                
            }
            gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][0] /= segmentsToAdd[i].cloud.size();
            gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][1] /= segmentsToAdd[i].cloud.size();
            gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][2] /= segmentsToAdd[i].cloud.size();

            // if (gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][2] < 0)
            // {
            //     printf("diag_cov: %f\n", gaussianMixtureModels.at(sectorId)->diagonal_covariance[i][2]);
            //     for (auto &pt : segmentsToAdd[i].cloud)
            //     {
            //         printf("z: %f, mean: %f cov: %f\n", pt.normal_z, gaussianMixtureModels.at(sectorId)->mean[i][2], ((pt.normal_z - gaussianMixtureModels.at(sectorId)->mean[i][2])*(pt.normal_z - gaussianMixtureModels.at(sectorId)->mean[i][2])));
            //     }
            // }
                    // printf("setGaussain: %d mean: %f; %f; %f\n", i, segmentsToAdd[i].meanR*cos(segmentsToAdd[i].meanTheta), segmentsToAdd[i].meanR*sin(segmentsToAdd[i].meanTheta), segmentsToAdd[i].meanZ);

        }
        printf("Gaussian Model Time: %f\n", tic_toc.toc());
    }

    void setGaussianMixtureModel(int sectorId, vector<Segment> segmentsToAdd)
    {
        TicToc tic_toc;
        int dataDim = 3;
        int numGauss = (int) segmentsToAdd.size();
        int numIter = 10;
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
            printf("setGaussain: %d mean: %f; %f; %f\n", i, segmentsToAdd[i].meanR*cos(segmentsToAdd[i].meanTheta), segmentsToAdd[i].meanR*sin(segmentsToAdd[i].meanTheta), segmentsToAdd[i].meanZ);
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

        // printf("sector Id: %d, numGauss: %d, numData:%d\n", sectorId, numGauss, numData);

        // printf("step	log_likelihood\n");
        for (int i = 0; i < numIter; i++)
        {
            double log_likelihood = gaussianMixtureModels.at(sectorId)->Expectaion_Maximization(numData, data);
            // if ((i + 1) % 10 == 0) printf("%d	%lf\n", i + 1, log_likelihood);
        }
        printf("Gaussian Model Time: %f\n", tic_toc.toc());
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
        }
        validSegments.insert(make_pair(sectorId, segmentsToAdd));
        // if (type_ == GLOBAL) {
            setGaussianModel(sectorId, segmentsToAdd);
            // setGaussianMixtureModel(sectorId, segmentsToAdd);
        // }
    }

    void init(int numSectors, double FOV, double maxR, double maxZ, Type type)
    {
        segIdCurr_ = 0;
        numSectors_ = numSectors;
        maxR_ = maxR;
        maxZ_ = maxZ;
        type_ = type;
        FOV_ = FOV * (M_PI / 180.0);
        thetaRes_ = FOV_ / numSectors; // [rad]
        
        gaussianMixtureModels.reserve(numSectors);
        sectors.reserve(numSectors);
        for (int i = 0; i < (int)numSectors; ++i)
        {
            pcl::PointCloud<PointType>::Ptr sectorCloud(new pcl::PointCloud<PointType>);
            boost::shared_ptr<Gaussian_Mixture_Model> GMM(new Gaussian_Mixture_Model);
            sectors.emplace_back(sectorCloud);
            gaussianMixtureModels.push_back(GMM);
        }

        kdTree_.reset(new pcl::search::KdTree<PointType>());
        clusterExtractor_.setClusterTolerance(1.0);
        clusterExtractor_.setMinClusterSize(5);
        clusterExtractor_.setMaxClusterSize(3000);
        clusterExtractor_.setSearchMethod(kdTree_);
    }

    void clear()
    {
        segIdCurr_ = 0;
        validSegments.clear();
        for (int i = 0; i < (int)numSectors_; ++i)
        {
            if (!sectors[i]->points.empty())
                sectors[i]->points.clear();
        }
    }
};



class Test : public ParamServer
{

public:

    ros::Subscriber subLaserCloud;
    ros::Subscriber subCloudInfo;
    ros::Subscriber subNonGroundCloud;
    ros::Publisher pubSegmentedCloudLocal;
    ros::Publisher pubCentroidCloudLocal;
    ros::Publisher pubSegmentedCloudGlobal;
    ros::Publisher pubCentroidCloudGlobal;
    ros::Publisher pubRayMap;
    ros::Publisher pubSegmentID;
    ros::Publisher pubGlobalMap;
    mutex mtxCloud;
    lvi_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;
    deque<pcl::PointCloud<PointType>> cloudQueue;
    deque<double> timeQueue;
    pcl::PointCloud<PointType>::Ptr depthCloud;

    RayMap localRayMap;
    RayMap globalRayMap;
    int numSectors;
    double maxR;
    double maxZ;
    double FOV;

    visualization_msgs::MarkerArray rayMapVisualization;

    Test()
    {
        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test", 5, &Test::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        // subCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, &Test::cloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subNonGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/GSeg/nonground_ptCloud", 5, &Test::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubSegmentedCloudLocal = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_segmented_cloud", 1);
        pubCentroidCloudLocal  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_centroids", 1);
        pubSegmentedCloudGlobal = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_segmented_cloud", 1);
        pubCentroidCloudGlobal  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_centroids", 1);
        pubGlobalMap        = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_cloud", 1);
        pubRayMap               = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/raymap", 1, true);
        pubSegmentID            = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/segment_id", 1);
        initializationValue();
    }

    void initializationValue()
    {
        depthCloud.reset(new pcl::PointCloud<PointType>());
        numSectors = 1;
        maxR = 100.0;
        maxZ = 1.5;
        FOV = 180.0; // [deg]
        localRayMap.init(numSectors, FOV, maxR, maxZ, LOCAL);
        globalRayMap.init(numSectors, FOV, maxR, maxZ, GLOBAL);
        rayMapVisualization = localRayMap.getRayMap();
        printf("rayMap initialized\n");
    }

    void publishSegmentedCloud(ros::Publisher *thisPubCloud, ros::Publisher *thisPubCentroid, RayMap raymap, ros::Time thisStamp, string thisFrame)
    {
        pcl::PointCloud<PointType2>::Ptr outPcl (new pcl::PointCloud<PointType2>);
        pcl::PointCloud<PointType>::Ptr centroids (new pcl::PointCloud<PointType>);
        sensor_msgs::PointCloud2 segmentedCloud;
        sensor_msgs::PointCloud2 centroidCloud;

        visualization_msgs::MarkerArray ids;

        for (const auto segs : raymap.validSegments)
        {
            for (int i = 0; i < (int) segs.second.size(); ++i)
            {
                Segment seg = segs.second[i];
                *outPcl += seg.cloud;
                PointType centroid;
                // centroid.x = seg.centroidX;
                // centroid.y = seg.centroidY;
                // centroid.z = seg.centroidZ;
                centroid.x = seg.meanR * cos(seg.meanTheta);
                centroid.y = seg.meanR * sin(seg.meanTheta);
                centroid.z = seg.meanZ;

                // if (raymap.getType() == GLOBAL)
                //     printf("Publish: %d mean: %f; %f; %f\n", i, centroid.x,  centroid.y, centroid.z);

                centroid.intensity = 100.0;
                centroids->points.emplace_back(centroid);
                
                std::ostringstream stream;
                stream.precision(3);
                stream << seg.segmentId << ": " << seg.staticProb;
                std::string new_string = stream.str();

                visualization_msgs::Marker text;
                text.header.frame_id = "base_link";
                text.scale.z = 0.5;
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
                text.color.a = 1.0;
                text.action = 0;
                text.type = 9; // TEXT_VIEW_FACING
                text.id = seg.segmentId;
                text.text = new_string;
                text.pose.position.x = centroid.x;
                text.pose.position.y = centroid.y;
                text.pose.position.z = centroid.z;
                text.pose.orientation.w = 1.0;
                ids.markers.push_back(text);
            }
        }

        if (raymap.getType() == LOCAL)
        {
            if (pubSegmentID.getNumSubscribers() != 0)
                pubSegmentID.publish(ids);
        }

        printf("outPCL size: %d\n", (int)outPcl->points.size());


        pcl::toROSMsg(*outPcl, segmentedCloud);
        pcl::toROSMsg(*centroids, centroidCloud);
        segmentedCloud.header.stamp = thisStamp;
        segmentedCloud.header.frame_id = thisFrame;
        centroidCloud.header = segmentedCloud.header;
        if (thisPubCloud->getNumSubscribers() != 0)
            thisPubCloud->publish(segmentedCloud);
        if (thisPubCentroid->getNumSubscribers() != 0)
            thisPubCentroid->publish(centroidCloud);
    }

    void inference2()
    {
        TicToc tic_toc;
        for (int i = 0; i < numSectors; ++i)
        {
            printf("-----------------------------------Sector %d\n", i);
            Gaussian_Mixture_Model global_gmm = *globalRayMap.gaussianMixtureModels[i];
            Gaussian_Mixture_Model local_gmm = *localRayMap.gaussianMixtureModels[i];
            printf("number of curr gaussian: %d\n", local_gmm.number_gaussian_components);
            printf("number of prev gaussian: %d\n", global_gmm.number_gaussian_components);
            if (global_gmm.number_gaussian_components == 0) // Case III or IV: no GMM model in the prev. frames
            {

            }
            else // Case I or II: GMM model exists in the prev. frames
            {
                if (localRayMap.validSegments[i].size() > 0) // Case I: current frame data & GMM model
                {
                    for (auto &seg : localRayMap.validSegments[i])
                    {
                        printf("-------------------------------------Segment\n");
                        // // 1. set data (entire segment)
                        // int numData = seg.cloud.points.size();
                        // double **data = new double*[numData];
                        // std::vector<double> prev_likelihoods;
                        // std::vector<double> curr_likelihoods;
                        // double max_prev_likelihood;
                        // double max_curr_likelihood;

                        // for (int k = 0; k < global_gmm.number_gaussian_components; ++k)
                        // {
                        //     double likelihood = 0.0;
                        //     for (int j = 0; j < numData; ++j)
                        //     {
                        //         data[j] = new double[3];
                        //         data[j][0] = (double)seg.cloud.points[j].normal_x;
                        //         data[j][1] = (double)seg.cloud.points[j].normal_y;
                        //         data[j][2] = (double)seg.cloud.points[j].normal_z; 
                        //         likelihood += global_gmm.Gaussian_Distribution(data[j], k);
                        //     }
                        //     // printf("Previous k: %d, likelihood: %f\n", k, likelihood);
                        //     prev_likelihoods.push_back(likelihood);
                        // }

                        // for (int k = 0; k < local_gmm.number_gaussian_components; ++k)
                        // {
                        //     double likelihood = 0.0;
                        //     for (int j = 0; j < numData; ++j)
                        //     {
                        //         data[j] = new double[3];
                        //         data[j][0] = (double)seg.cloud.points[j].normal_x;
                        //         data[j][1] = (double)seg.cloud.points[j].normal_y;
                        //         data[j][2] = (double)seg.cloud.points[j].normal_z; 
                        //         likelihood += local_gmm.Gaussian_Distribution(data[j], k);
                        //     }
                        //     // printf("Current k: %d, likelihood: %f\n", k, likelihood);
                        //     curr_likelihoods.push_back(likelihood);
                        // }
                        // // end

                        // 2. set data (centroid only)
                        double likelihood = 0.0;
                        int numData = 1;
                        double **data = new double*[numData];
                        data[0] = new double[3];
                        // data[0] = new double[2];
                        data[0][0] = seg.meanR;
                        data[0][1] = seg.meanTheta;
                        data[0][2] = seg.meanZ;
                        std::vector<double> prev_likelihoods;
                        std::vector<double> curr_likelihoods;
                        double max_prev_likelihood;
                        double max_curr_likelihood;
                        for (int k = 0; k < global_gmm.number_gaussian_components; ++k)
                        {
                            prev_likelihoods.push_back(global_gmm.Gaussian_Distribution(data[0], k));
                        }
                        for (int k = 0; k < local_gmm.number_gaussian_components; ++k)
                        {
                            curr_likelihoods.push_back(local_gmm.Gaussian_Distribution(data[0], k));
                        }
                        // end

                        max_prev_likelihood = *max_element(prev_likelihoods.begin(), prev_likelihoods.end());
                        max_curr_likelihood = *max_element(curr_likelihoods.begin(), curr_likelihoods.end());
                        double dynamic_likelihood = abs(max_prev_likelihood - max_curr_likelihood);
                        double static_likelihood = max_prev_likelihood;
                        double static_prob = static_likelihood / (dynamic_likelihood + static_likelihood);
                        seg.staticProb = static_prob;
                        printf("Seg %d: %f %f %f\n", seg.segmentId, max_prev_likelihood, max_curr_likelihood, static_prob);
                    }
                } 
                else // Case II: no current frame data but GMM model 
                {

                }
            }
        }
        printf("Inference Time: %f\n", tic_toc.toc());   
    }

    void inference()
    {
        TicToc tic_toc;
        for (int i = 0; i < numSectors; ++i)
        {
            printf("-----------------------------------Sector %d\n", i);
            Gaussian_Mixture_Model gmm = *globalRayMap.gaussianMixtureModels[i];
            printf("number of gaussian: %d\n", gmm.number_gaussian_components);
            if (gmm.number_gaussian_components == 0) // Case III or IV: no GMM model in the prev. frames
            {

            }
            else // Case I or II: GMM model exists in the prev. frames
            {
                if (localRayMap.validSegments[i].size() > 0) // Case I: current frame data & GMM model
                {
                    for (const auto seg : localRayMap.validSegments[i])
                    {
                        printf("-------------------------------------Segment\n");
                        // // set data (entire segment)
                        // double likelihood = 0.0;
                        // int numData = seg.cloud.points.size();
                        // double **data = new double*[numData];
                        // for (int j = 0; j < numData; ++j)
                        // {
                        //     data[j] = new double[3];
                        //     data[j][0] = (double)seg.cloud.points[j].normal_x;
                        //     data[j][1] = (double)seg.cloud.points[j].normal_y;
                        //     data[j][2] = (double)seg.cloud.points[j].normal_z;
                        //     for (int k = 0; k < gmm.number_gaussian_components; ++k)
                        //     {
                        //         likelihood += gmm.Gaussian_Distribution(data[j], k);
                        //     }
                        //     // likelihood += gmm.Calculate_Likelihood(data[j]);
                        // }

                        // set data (centroid only)
                        double likelihood = 0.0;
                        int numData = 1;
                        double **data = new double*[numData];
                        data[0] = new double[3];
                        data[0][0] = seg.meanR;
                        data[0][1] = seg.meanTheta;
                        data[0][2] = seg.meanZ;
                        for (int k = 0; k < gmm.number_gaussian_components; ++k)
                        {
                            likelihood += gmm.Gaussian_Distribution(data[0], k);
                        }

                        printf("Seg %d: %f\n", seg.segmentId, likelihood);
                    }
                } 
                else // Case II: no current frame data but GMM model 
                {

                }
            }
        }
        printf("Inference Time: %f\n", tic_toc.toc());
    }
    
    void segmentCloud()
    {

    }

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msgIn)
    {
        static int lidar_count = -1;
        if (++lidar_count % (LIDAR_SKIP+1) != 0)
            return;

        pubRayMap.publish(rayMapVisualization);
        // 0. listen to transform
        static tf::TransformListener listener;
        static tf::StampedTransform transform;
        try{
            listener.waitForTransform("odom", "base_link", msgIn->header.stamp, ros::Duration(0.01));
            listener.lookupTransform("odom", "base_link", msgIn->header.stamp, transform);
        } 
        catch (tf::TransformException ex){
            ROS_ERROR("lidar no tf");
            return;
        }
        printf("Transform \n");
        double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
        xCur = transform.getOrigin().x();
        yCur = transform.getOrigin().y();
        zCur = transform.getOrigin().z();
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(rollCur, pitchCur, yawCur);
        Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

        // 1. convert laser cloud message to pcl
        pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*msgIn, *laserCloudIn);

        // 2. downsample new cloud (save memory)
        pcl::PointCloud<PointType>::Ptr laserCloudInDS(new pcl::PointCloud<PointType>());
        static pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.setInputCloud(laserCloudIn);
        downSizeFilter.filter(*laserCloudInDS);
        *laserCloudIn = *laserCloudInDS;

        // 3. filter lidar points (only keep points in camera view)
        pcl::PointCloud<PointType>::Ptr laserCloudInFilter(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)laserCloudIn->size(); ++i)
        {
            PointType p = laserCloudIn->points[i];
            // if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
                laserCloudInFilter->push_back(p);
        }
        *laserCloudIn = *laserCloudInFilter;

        // 4. set dynamic likelihood
        localRayMap.clear();
        globalRayMap.clear();
        
        if (cloudQueue.size() > 0)
        {
            cloudQueue.clear();
            // 4.1 create ray map
            pcl::PointCloud<PointType>::Ptr laserCloudLocal(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*depthCloud, *laserCloudLocal, transNow.inverse());
            publishCloud(&pubGlobalMap, laserCloudLocal, cloudHeader.stamp, "base_link");
            localRayMap.setRayMap(laserCloudIn);
            globalRayMap.setRayMap(laserCloudLocal);
            inference2();
            publishSegmentedCloud(&pubSegmentedCloudLocal, &pubCentroidCloudLocal, localRayMap, cloudHeader.stamp, "base_link");
            publishSegmentedCloud(&pubSegmentedCloudGlobal, &pubCentroidCloudGlobal, globalRayMap, cloudHeader.stamp, "base_link");
        }
        
        // 5. transform new cloud into global odom frame
        pcl::PointCloud<PointType>::Ptr laserCloudGlobal(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*laserCloudIn, *laserCloudGlobal, transNow);

        // 6. save current cloud to global map
        double timeScanCur = msgIn->header.stamp.toSec();
        cloudQueue.push_back(*laserCloudGlobal);
        timeQueue.push_back(timeScanCur);

        // // 7. pop old cloud
        // while (!timeQueue.empty())
        // {
        //     if (timeScanCur - timeQueue.front() > 0.3)
        //     {
        //         cloudQueue.pop_front();
        //         timeQueue.pop_front();
        //     } else {
        //         break;
        //     }
        // }
        printf("cloudQueue size: %d\n", (int)cloudQueue.size());
        std::lock_guard<std::mutex> lock(mtxCloud);
        {
            // 8. fuse global cloud
            depthCloud->clear();
            for (int i = 0; i < (int)cloudQueue.size(); ++i)
                *depthCloud += cloudQueue[i];

            // 9. downsample global cloud
            pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.setInputCloud(depthCloud);
            downSizeFilter.filter(*depthCloudDS);
            *depthCloud = *depthCloudDS;
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");

    Test test;
   
    ros::spin();

    return 0;
}
