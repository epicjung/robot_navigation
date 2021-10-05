// #include "utility.h"
#include "lvi_sam/cloud_info.h"
#include "parameters.h"
#include "feature_tracker.h"
#include "GMM.h"

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

enum Type {LOCAL, GLOBAL};

struct RayMap
{
    vector<pcl::PointCloud<PointType2>::Ptr> sectors;
    pcl::EuclideanClusterExtraction<PointType2> clusterExtractor_;
    pcl::search::KdTree<PointType2>::Ptr kdTree_;
    vector<boost::shared_ptr<Gaussian_Mixture_Model>> gaussianMixtureModels;
    unordered_map<int, vector<Segment>> validSegments;
    std::vector<pcl::PointIndices> clusterIndices;

    int numSectors_;
    Type type_;
    double maxR_;
    double maxZ_;
    double thetaRes_;
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
        lines.header.frame_id = "vins_body_ros";
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
            text.header.frame_id = "vins_body_ros";
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

    void setRayMap(pcl::PointCloud<PointType2>::Ptr laserCloudIn)
    {
        // 1. build ray map for current scan
        printf("Total size: %d\n", (int)laserCloudIn->points.size());
        for (auto &pt : laserCloudIn->points)
        {
            if (pt.intensity == 1.0) // nonground
            {
                double radius = xy2radius(pt.x, pt.y);
                double theta = xy2theta(pt.x, pt.y);
                if (radius <= maxR_ && pt.z <= maxZ_ && abs(theta) <= FOV_ / 2.0)
                {
                    int sectorId = min(static_cast<int>((theta + FOV_ / 2.0) / thetaRes_), numSectors_ - 1);
                    sectors.at(sectorId)->points.push_back(pt);
                }
            }
        }

        // 2. Gaussian mixture odeling of the ray map
        clusterIndices.clear();
        for (int i = 0; i < (int)sectors.size(); ++i)
        {
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

    void addSegments(int sectorId, std::vector<pcl::PointIndices> segments, pcl::PointCloud<PointType2>::Ptr refCloud)
    {
        vector<Segment> segmentsToAdd;
        for (size_t i = 0u; i < segments.size(); ++i)
        {
            Segment segment;
            segment.segmentId = (int)i;
            std::vector<int> indices = segments[i].indices;
            for (size_t j = 0u; j < indices.size(); ++j)
            {
                const size_t index = indices[j];
                PointType2 point = refCloud->points[index];
                point.normal_x = xy2radius(point.x, point.y); // radial distance
                point.normal_y = xy2theta(point.x, point.y); // azimuth angle
                point.normal_z = point.z;
                point.intensity = (float) segment.segmentId;
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
            pcl::PointCloud<PointType2>::Ptr sectorCloud(new pcl::PointCloud<PointType2>);
            boost::shared_ptr<Gaussian_Mixture_Model> GMM(new Gaussian_Mixture_Model);
            sectors.emplace_back(sectorCloud);
            gaussianMixtureModels.push_back(GMM);
        }

        kdTree_.reset(new pcl::search::KdTree<PointType2>());
        clusterExtractor_.setClusterTolerance(1.0);
        clusterExtractor_.setMinClusterSize(5);
        clusterExtractor_.setMaxClusterSize(3000);
        clusterExtractor_.setSearchMethod(kdTree_);
    }

    void clear()
    {
        validSegments.clear();
        for (int i = 0; i < (int)numSectors_; ++i)
        {
            if (!sectors[i]->points.empty())
                sectors[i]->points.clear();
        }
    }
};


#define SHOW_UNDISTORTION 0

class Test
{

public:

    // global depth register for obtaining depth of a feature
    DepthRegister *depthRegister;

    // feature publisher for VINS estimator
    ros::NodeHandle nh;
    ros::Subscriber sub_img;
    ros::Publisher pub_feature;
    ros::Publisher pub_match;
    ros::Publisher pub_restart;
    ros::Publisher pub_cloud;
    ros::Publisher pub_cloud2;
    ros::Publisher pub_cloud3;

    // feature tracker variables
    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    ros::Subscriber subLaserCloud;
    ros::Subscriber subCloud;
    ros::Subscriber subNonGroundCloud;
    ros::Publisher pubSegmentedCloudLocal;
    ros::Publisher pubCentroidCloudLocal;
    ros::Publisher pubSegmentedCloudGlobal;
    ros::Publisher pubCentroidCloudGlobal;
    ros::Publisher pubRayMap;
    ros::Publisher pubSegmentID;
    ros::Publisher pubGlobalMap;
    mutex mtx_lidar;
    lvi_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;
    deque<pcl::PointCloud<PointType>> cloudQueue;
    deque<pcl::PointCloud<PointType2>> nonGroundQueue;
    deque<pcl::PointCloud<PointType2>> groundQueue;
    deque<double> timeQueue;
    pcl::PointCloud<PointType2>::Ptr depthCloud;
    pcl::PointCloud<PointType2>::Ptr nonGroundCloud;

    RayMap localRayMap;
    RayMap globalRayMap;
    int numSectors;
    double maxR;
    double maxZ;
    double FOV;

    visualization_msgs::MarkerArray rayMapVisualization;

    Test()
    {
        readParameters(nh);

        // subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/feature/cloud_test", 5, &Test::laserCloudHandler, this, ros::TransportHints().tcpNoDelay());
        // subCloudInfo = nh.subscribe<lvi_sam::cloud_info>(PROJECT_NAME + "/lidar/deskew/cloud_info", 5, &Test::cloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // subNonGroundCloud = nh.subscribe<sensor_msgs::PointCloud2>("/GSeg/nonground_ptCloud", 5, &Test::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subCloud          = nh.subscribe<sensor_msgs::PointCloud2>("/GSeg/ground_nonground_ptCloud", 5, &Test::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        pubSegmentedCloudLocal = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_segmented_cloud", 1);
        pubCentroidCloudLocal  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_centroids", 1);
        pubSegmentedCloudGlobal = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_segmented_cloud", 1);
        pubCentroidCloudGlobal  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_centroids", 1);
        pubGlobalMap        = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_cloud", 1);
        pubRayMap               = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/raymap", 1, true);
        pubSegmentID            = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/segment_id", 1);
        
        // feature tracker
        sub_img     = nh.subscribe(IMAGE_TOPIC,       5,    &Test::imgHandler, this, ros::TransportHints().tcpNoDelay());
        pub_restart = nh.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
        pub_match   = nh.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
        pub_feature = nh.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);

        initializationValue();
    }

    void initializationValue()
    {
        depthCloud.reset(new pcl::PointCloud<PointType2>());
        nonGroundCloud.reset(new pcl::PointCloud<PointType2>());
        numSectors = 1;
        maxR = 150.0;
        maxZ = 10.0;
        FOV = 340.0; // [deg]
        localRayMap.init(numSectors, FOV, maxR, maxZ, LOCAL);
        globalRayMap.init(numSectors, FOV, maxR, maxZ, GLOBAL);
        rayMapVisualization = localRayMap.getRayMap();

        printf("rayMap initialized\n");

        // read camera params
        for (int i = 0; i < NUM_OF_CAM; i++)
            trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

        // load fisheye mask to remove features on the boundry
        if(FISHEYE)
        {
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
                if(!trackerData[i].fisheye_mask.data)
                {
                    ROS_ERROR("load fisheye mask fail");
                    ROS_BREAK();
                }
                else
                    ROS_INFO("load mask success");
            }
        }

        // initialize depthRegister (after readParameters())
        depthRegister = new DepthRegister(nh);
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
                text.header.frame_id = "vins_body_ros";
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

    void inference2(pcl::PointCloud<PointType2>::Ptr cloudIn)
    {
        cloudIn->clear();
        TicToc tic_toc;
        Gaussian_Mixture_Model global_gmm = *globalRayMap.gaussianMixtureModels[0];
        Gaussian_Mixture_Model local_gmm = *localRayMap.gaussianMixtureModels[0];
        printf("number of curr gaussian: %d\n", local_gmm.number_gaussian_components);
        printf("number of prev gaussian: %d\n", global_gmm.number_gaussian_components);
        if (global_gmm.number_gaussian_components == 0) // Case III or IV: no GMM model in the prev. frames
        {

        }
        else // Case I or II: GMM model exists in the prev. frames
        {
            if (localRayMap.validSegments[0].size() > 0) // Case I: current frame data & GMM model
            {
                for (auto &seg : localRayMap.validSegments[0])
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
                    std::vector<int> indices = localRayMap.clusterIndices.at(seg.segmentId).indices;
                    printf("Seg %d: %d, %f\n", seg.segmentId, (int)indices.size(), static_prob);
                    for (size_t j = 0u; j < indices.size(); ++j)
                    {
                        const size_t index = indices[j];
                        PointType2 p = localRayMap.sectors[0]->points[index];
                        p.curvature = (float)static_prob;
                        cloudIn->points.push_back(p);
                    }
                }
            } 
            else // Case II: no current frame data but GMM model 
            {

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

    void imgHandler(const sensor_msgs::ImageConstPtr &img_msg)
    {
        double cur_img_time = img_msg->header.stamp.toSec();

        if(first_image_flag)
        {
            first_image_flag = false;
            first_image_time = cur_img_time;
            last_image_time = cur_img_time;
            return;
        }
        // detect unstable camera stream
        if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
        {
            ROS_WARN("image discontinue! reset the feature tracker!");
            first_image_flag = true; 
            last_image_time = 0;
            pub_count = 1;
            std_msgs::Bool restart_flag;
            restart_flag.data = true;
            pub_restart.publish(restart_flag);
            return;
        }
        last_image_time = cur_img_time;
        // frequency control
        if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            // reset the frequency control
            if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = cur_img_time;
                pub_count = 0;
            }
        }
        else
        {
            PUB_THIS_FRAME = false;
        }

        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat show_img = ptr->image;
        TicToc t_r;
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            ROS_DEBUG("processing camera %d", i);
            if (i != 1 || !STEREO_TRACK)
                trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), cur_img_time);
            else
            {
                if (EQUALIZE)
                {
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                    clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                }
                else
                    trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
            }

            #if SHOW_UNDISTORTION
                trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
            #endif
        }

        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < NUM_OF_CAM; j++)
                if (j != 1 || !STEREO_TRACK)
                    completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }

        if (PUB_THIS_FRAME)
        {
            pub_count++;
            sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
            sensor_msgs::ChannelFloat32 id_of_point;
            sensor_msgs::ChannelFloat32 u_of_point;
            sensor_msgs::ChannelFloat32 v_of_point;
            sensor_msgs::ChannelFloat32 velocity_x_of_point;
            sensor_msgs::ChannelFloat32 velocity_y_of_point;

            feature_points->header.stamp = img_msg->header.stamp;
            feature_points->header.frame_id = "vins_body";

            vector<set<int>> hash_ids(NUM_OF_CAM);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                auto &un_pts = trackerData[i].cur_un_pts;
                auto &cur_pts = trackerData[i].cur_pts;
                auto &ids = trackerData[i].ids;
                auto &pts_velocity = trackerData[i].pts_velocity;
                for (unsigned int j = 0; j < ids.size(); j++)
                {
                    if (trackerData[i].track_cnt[j] > 1)
                    {
                        int p_id = ids[j];
                        hash_ids[i].insert(p_id);
                        geometry_msgs::Point32 p;
                        p.x = un_pts[j].x;
                        p.y = un_pts[j].y;
                        p.z = 1;

                        feature_points->points.push_back(p);
                        id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                        u_of_point.values.push_back(cur_pts[j].x);
                        v_of_point.values.push_back(cur_pts[j].y);
                        velocity_x_of_point.values.push_back(pts_velocity[j].x);
                        velocity_y_of_point.values.push_back(pts_velocity[j].y);
                    }
                }
            }

            feature_points->channels.push_back(id_of_point);
            feature_points->channels.push_back(u_of_point);
            feature_points->channels.push_back(v_of_point);
            feature_points->channels.push_back(velocity_x_of_point);
            feature_points->channels.push_back(velocity_y_of_point);

            // get feature depth from lidar point cloud
            pcl::PointCloud<PointType2>::Ptr depth_cloud_temp(new pcl::PointCloud<PointType2>());
            mtx_lidar.lock();
            *depth_cloud_temp = *depthCloud;
            mtx_lidar.unlock();

            printf("Depth point: %f \n", img_msg->header.stamp.toSec());
            std::pair<sensor_msgs::ChannelFloat32, sensor_msgs::ChannelFloat32> depth_and_prob = depthRegister->get_depth_and_prob(img_msg->header.stamp, show_img, depth_cloud_temp, trackerData[0].m_camera, feature_points);
            sensor_msgs::ChannelFloat32 depth_of_points = depth_and_prob.first;
            sensor_msgs::ChannelFloat32 prob_of_points = depth_and_prob.second;
            feature_points->channels.push_back(depth_of_points);
            feature_points->channels.push_back(prob_of_points);

            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                init_pub = 1;
            }
            else
                pub_feature.publish(feature_points);

            // publish features in image
            if (pub_match.getNumSubscribers() != 0)
            {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
                //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
                cv::Mat stereo_img = ptr->image;

                for (int i = 0; i < NUM_OF_CAM; i++)
                {
                    cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                    for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                    {
                        if (SHOW_TRACK)
                        {
                            // track count
                            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                            cv::circle(tmp_img, trackerData[i].cur_pts[j], 8, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                        } else {
                            // depth 
                            if(j < depth_of_points.values.size())
                            {
                                std::ostringstream stream;
                                stream.precision(3);
                                stream << trackerData[i].ids[j] << ":" << prob_of_points.values[j];
                                std::string out_string = stream.str();
                                if (depth_of_points.values[j] > 0)
                                {
                                    if (j < prob_of_points.values.size())
                                    {

                                    }
                                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 8, cv::Scalar(0, 255, 0), 4);
                                    cv::putText(tmp_img, out_string, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 0, 0));

                                }
                                else
                                {
                                    // cv::circle(tmp_img, trackerData[i].cur_pts[j], 8, cv::Scalar(0, 0, 255), 4);
                                }
                            }
                        }
                    }
                }
                pub_match.publish(ptr->toImageMsg());
            }
        }
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
            listener.waitForTransform("vins_world", "vins_body_ros", msgIn->header.stamp, ros::Duration(0.01));
            listener.lookupTransform("vins_world", "vins_body_ros", msgIn->header.stamp, transform);
            // listener.waitForTransform("odom", "base_link", msgIn->header.stamp, ros::Duration(0.01));
            // listener.lookupTransform("odom", "base_link", msgIn->header.stamp, transform);
        } 
        catch (tf::TransformException ex){
            ROS_ERROR("lidar no tf");
            return;
        }
        printf("cloudHandler: %f \n", msgIn->header.stamp.toSec());
        double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
        xCur = transform.getOrigin().x();
        yCur = transform.getOrigin().y();
        zCur = transform.getOrigin().z();
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(rollCur, pitchCur, yawCur);
        Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

        // 0.1 get extrinsic (translation only by assuming z (cam) = x (lidar)) from cam to lidar
        Eigen::Affine3f transLidar2Cam = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, 0.0, 0.0, 0.0);
        transNow = transNow * transLidar2Cam; // t_world_lidar * t_lidar_cam = t_world_cam

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

        // // 3. filter lidar points (only keep points in camera view)
        // pcl::PointCloud<PointType>::Ptr laserCloudInFilter(new pcl::PointCloud<PointType>());
        // for (int i = 0; i < (int)laserCloudIn->size(); ++i)
        // {
        //     PointType p = laserCloudIn->points[i];
        //     // if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
        //         laserCloudInFilter->push_back(p);
        // }
        // *laserCloudIn = *laserCloudInFilter;

        // 3.1 Separates cloud into nonground and ground
        pcl::PointCloud<PointType2>::Ptr nonGroundLocal(new pcl::PointCloud<PointType2>());
        pcl::PointCloud<PointType2>::Ptr groundLocal(new pcl::PointCloud<PointType2>());
        for (auto pt : laserCloudIn->points)
        {
            PointType2 p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            if (p.intensity == 1.0) // nonground
            {  
                if (REMOVE_DYNAMIC)
                    p.curvature = 0.0; // static probability
                nonGroundLocal->points.push_back(p);
            }
            else
            {
                if (REMOVE_DYNAMIC)
                    p.curvature = 1.0; // static probability
                groundLocal->points.push_back(p);
            }
        }
        printf("Ground: %d, Nonground: %d\n", (int)groundLocal->points.size(),(int)nonGroundLocal->points.size()); 

        // 4. set dynamic likelihood
        localRayMap.clear();
        globalRayMap.clear();
        
        if (nonGroundQueue.size() > 0)
        {
            nonGroundQueue.clear();
            groundQueue.clear();
            // 4.1 create ray map
            if (REMOVE_DYNAMIC)
            {
                pcl::PointCloud<PointType2>::Ptr laserCloudLocal(new pcl::PointCloud<PointType2>());
                pcl::transformPointCloud(*nonGroundCloud, *laserCloudLocal, transNow.inverse());
                publishCloud(&pubGlobalMap, laserCloudLocal, cloudHeader.stamp, "vins_body_ros");
                localRayMap.setRayMap(nonGroundLocal);
                globalRayMap.setRayMap(laserCloudLocal);
                inference2(nonGroundLocal);
                publishSegmentedCloud(&pubSegmentedCloudLocal, &pubCentroidCloudLocal, localRayMap, cloudHeader.stamp, "vins_body_ros");
                publishSegmentedCloud(&pubSegmentedCloudGlobal, &pubCentroidCloudGlobal, globalRayMap, cloudHeader.stamp, "vins_body_ros");
            }
           
        }
        
        // 5. transform new cloud into global odom frame
        // pcl::PointCloud<PointType>::Ptr laserCloudGlobal(new pcl::PointCloud<PointType>());
        // pcl::transformPointCloud(*laserCloudIn, *laserCloudGlobal, transNow);
        pcl::PointCloud<PointType2>::Ptr nonGroundGlobal(new pcl::PointCloud<PointType2>());
        pcl::PointCloud<PointType2>::Ptr groundGlobal(new pcl::PointCloud<PointType2>());
        pcl::transformPointCloud(*nonGroundLocal, *nonGroundGlobal, transNow);
        pcl::transformPointCloud(*groundLocal, *groundGlobal, transNow);

        // 6. save current cloud to global map
        double timeScanCur = msgIn->header.stamp.toSec();
        groundQueue.push_back(*groundGlobal);
        nonGroundQueue.push_back(*nonGroundGlobal);
        timeQueue.push_back(timeScanCur);

        std::lock_guard<std::mutex> lock(mtx_lidar);
        {
            // 8. fuse global cloud
            depthCloud->clear();
            nonGroundCloud->clear();

            for (int i = 0; i < (int)groundQueue.size(); ++i)
            {
                *depthCloud += groundQueue[i];
                *depthCloud += nonGroundQueue[i];
                *nonGroundCloud += nonGroundQueue[i];
            }

            // check
            int under = 0;
            int over = 0;
            int ground = 0;
            for (auto p : depthCloud->points)
            {
                if (p.curvature < 0.7)
                    under++;
                else if (p.curvature >= 0.7 && p.curvature < 1.0)
                    over++;
                else
                    ground++;
            }
            printf("Under: %d, over: %d, ground: %d\n", under, over, ground);
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");

    Test test;

    ROS_INFO("\033[1;32m----> Test Started.\033[0m");
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    ros::MultiThreadedSpinner spinner(2);

    spinner.spin();

    return 0;
}
