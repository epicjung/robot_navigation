#include "utility.h"
#include "lvi_sam/cloud_info.h"
#include "TMAP/patchwork_obstacle.hpp"
#include "gmphd_filter.h"

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// // Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

using namespace gmphd;

const int queueLength = 500;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    boost::shared_ptr<Patchwork_M> patchwork_m;
    ClusterMap query_map;
    ClusterMap ref_map;
    std::deque<pcl::PointCloud<PointType>> deskewedCloudQueue;
    std::deque<ros::Time> timeQueue;
    ros::Publisher pub_static_scene;
    ros::Publisher pub_ref_clustered;
    ros::Publisher pub_ref_centroid;
    ros::Publisher pub_query_clustered;
    ros::Publisher pub_query_centroid;
    ros::Publisher pub_cluster_map;
    ros::Publisher pub_bbox;
    ros::Publisher pub_ref_id;
    ros::Publisher pub_query_id;
    bool tracker_flag;

    sensor_msgs::PointCloud2 currentCloudMsg;
    
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lvi_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanNext;
    std_msgs::Header cloudHeader;

    GMPHD<2> target_tracker;

public:
    ImageProjection():
    deskewFlag(0), tracker_flag(false)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>        (imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>      (PROJECT_NAME + "/vins/odometry/imu_propagate_ros", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> (PROJECT_NAME + "/lidar/deskew/cloud_deskewed", 5);
        pubLaserCloudInfo = nh.advertise<lvi_sam::cloud_info>      (PROJECT_NAME + "/lidar/deskew/cloud_info", 5);

        pub_query_clustered = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_segmented_cloud", 1);
        pub_query_centroid  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/local_centroids", 1);
        pub_ref_clustered = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_segmented_cloud", 1);
        pub_ref_centroid  = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/global_centroids", 1);
        pub_static_scene = nh.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/lidar/static_cloud", 1);
        pub_cluster_map         = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/raymap", 1, true);
        pub_query_id      = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/segment_id", 1);
        pub_ref_id     = nh.advertise<visualization_msgs::MarkerArray>(PROJECT_NAME + "/lidar/segment_id2", 1);
        pub_bbox       = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(PROJECT_NAME + "/lidar/bbox", 1);
        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        patchwork_m.reset(new Patchwork_M(&nh));
        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        query_map.init(QUERY, maxR, maxZ, clusteringTolerance, minClusterSize, maxClusterSize);
        ref_map.init(REFERENCE, maxR, maxZ, clusteringTolerance, minClusterSize, maxClusterSize);
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();
        
        cloudExtraction();

        tracking();

        // dynamicRemoval();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;
        else
        {
            currentCloudMsg = cloudQueue.front();
            cloudQueue.pop_front();

            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanNext = cloudQueue.front().header.stamp.toSec();
        }

        // convert cloud
        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

        // // rslidar
        // float verticalAngle, horizonAngle, range;
        // size_t rowInd, columnIdn, index, cloudSize;
        // cloudSize = laserCloudIn->points.size();
        // std::vector<float> angles{-25.0, -14.638, -10.281, -7.910, -6.424, -5.407, -4.667, -4.333, -4.000, -3.667, -3.333, -3.000, -2.667, -2.333, -2.000, -1.667, -1.333, -1.000, -0.667, -0.333, 0.000,
        //                           0.333, 0.667, 1.000, 1.333, 1.667, 2.333, 3.333, 4.667, 7.000, 10.333, 15.000};

        // for (pcl::PointCloud<PointXYZIRT>::iterator it = laserCloudIn->begin(); it != laserCloudIn->end(); it++)
        // {
        //     if (isnan(it->x) || isnan(it->y) || isnan(it->z))
        //     {
        //         laserCloudIn->erase(it);
        //     }
        //     else
        //     {
        //         PointXYZIRT thisPoint;
        //         thisPoint.x = it->x;
        //         thisPoint.y = it->y;
        //         thisPoint.z = it->z;
        //         verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y))*180/M_PI;
        //         auto iter_geq = std::lower_bound(angles.begin(), angles.end(), verticalAngle);
        //         if (iter_geq == angles.begin())
        //         {
        //             rowInd = 0;
        //         }
        //         else
        //         {
        //             float a = *(iter_geq - 1);
        //             float b = *(iter_geq);
        //             if (fabs(verticalAngle-a) < fabs(verticalAngle-b)){
        //                 rowInd = iter_geq - angles.begin() - 1;
        //             } else {
        //                 rowInd = iter_geq - angles.begin();
        //             }
        //         }                    
        //         it->ring = rowInd;
        //     }
        // }
        // laserCloudIn->is_dense = true;
        
        for (pcl::PointCloud<PointXYZIRT>::iterator it = laserCloudIn->begin(); it != laserCloudIn->end(); it++)
        {
            if (isnan(it->x) || isnan(it->y) || isnan(it->z))
            {
                laserCloudIn->erase(it);
            }
        }
        laserCloudIn->is_dense = true;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel (comment this for rslidar)
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }     

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == timeField)
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.odomX = startOdomMsg.pose.pose.position.x;
        cloudInfo.odomY = startOdomMsg.pose.pose.position.y;
        cloudInfo.odomZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.odomRoll  = roll;
        cloudInfo.odomPitch = pitch;
        cloudInfo.odomYaw   = yaw;
        cloudInfo.odomResetId = (int)round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanNext)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            return;

        float ratio = relTime / (timeScanNext - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = (int)laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            float range = pointDistance(thisPoint);
            
            if (range < 1.0)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // for the amsterdam dataset
            // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
            //     continue;
            // if (thisPoint.z < -2.0)
            //     continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
            // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

            int index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }

    void resetViz(ros::Time this_stamp, std::string this_frame)
    {
        visualization_msgs::MarkerArray deleter;
        visualization_msgs::Marker deleter_marker;
        deleter_marker.header.frame_id = this_frame;
        deleter_marker.header.stamp = this_stamp;
        deleter_marker.action = visualization_msgs::Marker::DELETEALL;
        deleter.markers.push_back(deleter_marker);
        pub_query_id.publish(deleter);
        pub_ref_id.publish(deleter);

        jsk_recognition_msgs::BoundingBoxArray bbox_array;
        bbox_array.header.stamp = this_stamp;
        bbox_array.header.frame_id = this_frame;
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.stamp = this_stamp;
        bbox.header.frame_id = this_frame;
        bbox_array.boxes.push_back(bbox);
        pub_bbox.publish(bbox_array);
    }

    void tracking()
    {
        static int lidar_count = -1;
        static int used_id_cnt = 0;
        if (++lidar_count % (0+1) != 0)
            return;
        query_map.clear();

        resetViz(cloudHeader.stamp, "odom");

        // 1. Ground segmentation
        pcl::PointCloud<PointType>::Ptr query_nonground(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr query_ground(new pcl::PointCloud<PointType>());
        double patchwork_process_time;
        patchwork_m->estimate_ground(*extractedCloud, *query_ground, *query_nonground, patchwork_process_time);
        printf("[Query] Ground: %d, Nonground: %d, time: %f\n", (int)query_ground->points.size(), (int)query_nonground->points.size(), patchwork_process_time);
        
        // 3. Find reference scan and transform
        static tf::TransformListener listener;
        static tf::StampedTransform init_transform;

        // TO-DO: "vins_world", "vins_body_ros" 
        try{
            listener.waitForTransform("odom", "base_link", cloudHeader.stamp, ros::Duration(0.01));
            listener.lookupTransform("odom", "base_link", cloudHeader.stamp, init_transform);
        } 
        catch (tf::TransformException ex){
            ROS_ERROR("no vins tf");
            return;
        }

        // Voxelize
        pcl::PointCloud<PointType>::Ptr query_nonground_ds(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());

        TicToc voxel_time;
        static pcl::VoxelGrid<PointType> query_downsize_filter;
        static pcl::PassThrough<PointType> pass;
        pass.setInputCloud(query_nonground);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(minZ, maxZ);
        pass.filter(*cloud_filtered);
        *query_nonground = *cloud_filtered;
        // query_downsize_filter.setSaveLeafLayout(true);
        query_downsize_filter.setLeafSize(nongroundDownsample, nongroundDownsample, nongroundDownsample);
        query_downsize_filter.setInputCloud(query_nonground);
        query_downsize_filter.filter(*query_nonground_ds);
        ROS_WARN("Voxelize: %f ms\n", voxel_time.toc());
        printf("After voxelized: %d\n", (int)query_nonground_ds->points.size());

        // get query pose
        double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
        xCur = init_transform.getOrigin().x();
        yCur = init_transform.getOrigin().y();
        zCur = init_transform.getOrigin().z();
        tf::Matrix3x3 init_m(init_transform.getRotation());
        init_m.getRPY(rollCur, pitchCur, yawCur);
        Eigen::Affine3f query_pose = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur); // T_world_query
        pcl::transformPointCloud(*query_nonground_ds, *query_nonground_ds, query_pose);

        TicToc tic_toc;
        query_map.buildMap(query_nonground_ds, used_id_cnt);
        // ROS_WARN("Id: %d Cluster map building: %f ms\n", used_id_cnt, tic_toc.toc());

        // init target tracking
        TicToc tracking_time;
        std::vector<Cluster> query_clusters = query_map.getMap();
        int const n_targets = (int)query_clusters.size();

        if (!tracker_flag)
        {
            GaussianModel<4> birth;
            birth.m_weight = 0.2f;
            birth.m_mean(0, 0) = 400.f;
            birth.m_mean(1, 0) = 400.f;
            birth.m_mean(2, 0) = 0.f;
            birth.m_mean(3, 0) = 0.f;
            birth.m_cov = (400.f * 400.f) * MatrixXf::Identity(4, 4);

            GaussianMixture<4> birth_model({birth});
            target_tracker.setBirthModel(birth_model);

            // Dynamics (motion model)
            target_tracker.setDynamicsModel(0.1f, 15.0f);

            // Detection model
            float const prob_detection = 0.98;
            float const meas_noise_pose = 0.5f;
            float const meas_noise_speed = 20.0f;
            float const meas_background = 0.02; // false detection prob
            target_tracker.setObservationModel(prob_detection, meas_noise_pose, meas_noise_speed, meas_background);
        
            // Pruning parameters
            target_tracker.setPruningParameters(0.1f, 0.0f, 100);

            // Spawn
            SpawningModel<4> spawn_model;
            std::vector<SpawningModel<4>> spawns = {spawn_model};
            target_tracker.setSpawnModel(spawns);

            // Survival over time
            target_tracker.setSurvivalProbability(0.4f); 

            tracker_flag = true;
        }

        // Track the circling targets
        std::vector<Target<2>> target_meas;
        Matrix<float, 2, 1> measurements;
        float const detection_prob = 1.0;
        int detected = 0;
        for (unsigned int i = 0; i < n_targets; ++i)
        {
            int const maxRand = detection_prob * RAND_MAX;
            if (rand() < maxRand)
            {
                if (query_clusters[i].bbox.value >= 0)
                {
                    measurements[0] = query_clusters[i].bbox.pose.position.x;
                    measurements[1] = query_clusters[i].bbox.pose.position.y;
                    target_meas.push_back({.position=measurements, .speed={0., 0.}, .weight=1., .id=query_clusters[i].id});
                    detected++;
                }
            }
        }
        printf("Detections: %d out of %d\n", detected, (int)query_clusters.size());
        target_tracker.setNewMeasurements(target_meas);
        target_tracker.propagate();
        const auto filtered=target_tracker.getTrackedTargets(0.1f);
        
        ClusterMap filtered_map;
        filtered_map.init(REFERENCE, 100.0, 1.5, 0.0, 0, 0);
        std::vector<Cluster> clusters;
        for (size_t i = 0; i < filtered.size(); i++)
        {
            Cluster cluster;
            cluster.id = filtered[i].id;
            cluster.centroid_x = filtered[i].position[0];
            cluster.centroid_y = filtered[i].position[1];
            cluster.feature = sqrt(pow(filtered[i].speed[0], 2) + pow(filtered[i].speed[1], 2));
            clusters.push_back(cluster);
            printf("id: %d, filtered: %f;%f;%f, weight: %f\n", filtered[i].id, filtered[i].position[0], filtered[i].position[1], 0.0, filtered[i].weight);
        }
        printf("meas size: %d, filtered size: %d\n", (int)target_meas.size(), (int)filtered.size());
        ROS_WARN("Tracking time: %f ms\n", tracking_time.toc());
        filtered_map.cluster_map_ = clusters;
        publishClusteredCloud(&pub_query_clustered, &pub_query_centroid, query_map, cloudHeader.stamp, "odom");
        publishClusteredCloud(&pub_ref_clustered, &pub_ref_centroid, filtered_map, cloudHeader.stamp, "odom");
        publishBoundingBox(&pub_bbox, query_map, cloudHeader.stamp, "odom");
    }

    void dynamicRemoval()
    {
        static int used_id_cnt = 0;
        query_map.clear();
        ref_map.clear();

        // 1. Ground segmentation
        pcl::PointCloud<PointType>::Ptr query_nonground(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr query_ground(new pcl::PointCloud<PointType>());
        double patchwork_process_time;
        patchwork_m->estimate_ground(*extractedCloud, *query_ground, *query_nonground, patchwork_process_time);
        printf("[Query] Ground: %d, Nonground: %d\n", (int)query_ground->points.size(), (int)query_nonground->points.size());
        
        // 2. push time and segmented cloud to queue
        deskewedCloudQueue.push_back(*query_nonground);
        timeQueue.push_back(cloudHeader.stamp);

        // 3. Find reference scan and transform
        static tf::TransformListener listener;
        static tf::StampedTransform init_transform;
        static tf::StampedTransform opt_transform;

        pcl::PointCloud<PointType>::Ptr ref_cloud(new pcl::PointCloud<PointType>());
        if (deskewedCloudQueue.size() == 5)
        {
            *ref_cloud = deskewedCloudQueue.front();
            ros::Time ref_time = timeQueue.front();
            deskewedCloudQueue.pop_front();
            timeQueue.pop_front();

            // TO-DO: "vins_world", "vins_body_ros" 
            try{
                listener.waitForTransform("odom", "base_link", cloudHeader.stamp, ros::Duration(0.01));
                listener.lookupTransform("odom", "base_link", cloudHeader.stamp, init_transform);
            } 
            catch (tf::TransformException ex){
                ROS_ERROR("no vins tf");
                return;
            }

            try {
                listener.waitForTransform("odom", "base_link", ref_time, ros::Duration(0.01));
                listener.lookupTransform("odom", "base_link", ref_time, opt_transform);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("no lidar tf");
                return;
            }
        }

        // 5. Compare query and reference scan
        if (ref_cloud->points.size() > 0)
        {
            // create new pointcloud with nonground points start from the first index
            pcl::PointCloud<PointType>::Ptr to_be_segmented(new pcl::PointCloud<PointType>());
            to_be_segmented->reserve(extractedCloud->points.size());
            size_t nonground_size = query_nonground->points.size();
            for (size_t i = 0; i < query_nonground->points.size(); i++)
                to_be_segmented->points.push_back(query_nonground->points[i]);
            
            for (size_t i = 0; i < query_ground->points.size(); i++)
                to_be_segmented->points.push_back(query_ground->points[i]);

            // get query pose
            double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
            xCur = init_transform.getOrigin().x();
            yCur = init_transform.getOrigin().y();
            zCur = init_transform.getOrigin().z();
            tf::Matrix3x3 init_m(init_transform.getRotation());
            init_m.getRPY(rollCur, pitchCur, yawCur);
            Eigen::Affine3f query_pose = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur); // T_world_query

            // get reference pose
            xCur = opt_transform.getOrigin().x();
            yCur = opt_transform.getOrigin().y();
            zCur = opt_transform.getOrigin().z();
            tf::Matrix3x3 opt_m(opt_transform.getRotation());
            opt_m.getRPY(rollCur, pitchCur, yawCur);
            Eigen::Affine3f ref_pose = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur); // T_world_ref
            printf("[REF] Nonground: %d\n", (int)ref_cloud->points.size());

            // Transform reference pointcloud to query pose
            Eigen::Affine3f rel_pose = query_pose.inverse() * ref_pose; // T_query_ref = (T_world_query)' * T_world_ref
            pcl::transformPointCloud(*ref_cloud, *ref_cloud, rel_pose);

            // Voxelize
            pcl::PointCloud<PointType>::Ptr query_nonground_ds(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr ref_nonground_ds(new pcl::PointCloud<PointType>());

            TicToc voxel_time;
            static pcl::VoxelGrid<PointType> query_downsize_filter;
            static pcl::VoxelGrid<PointType> ref_downsize_filter;
            query_downsize_filter.setSaveLeafLayout(true);
            query_downsize_filter.setLeafSize(nongroundDownsample, nongroundDownsample, nongroundDownsample);
            query_downsize_filter.setInputCloud(query_nonground);
            query_downsize_filter.filter(*query_nonground_ds);
            ref_downsize_filter.setLeafSize(nongroundDownsample, nongroundDownsample, nongroundDownsample);
            ref_downsize_filter.setInputCloud(ref_cloud);
            ref_downsize_filter.filter(*ref_nonground_ds);
            printf("Voxelize: %f ms\n", voxel_time.toc());

            TicToc tic_toc;
            query_map.buildMap(query_nonground_ds, used_id_cnt);
            ref_map.buildMap(ref_nonground_ds, used_id_cnt);
            printf("Cluster map building: %f ms\n", tic_toc.toc());

            // remove
            TicToc removal_time;
            inference(query_nonground_ds, to_be_segmented, query_downsize_filter, nonground_size);
            publishCloud(&pub_static_scene, to_be_segmented, cloudHeader.stamp, "base_link");
            publishClusteredCloud(&pub_query_clustered, &pub_query_centroid, query_map, cloudHeader.stamp, "base_link");
            publishClusteredCloud(&pub_ref_clustered, &pub_ref_centroid, ref_map, cloudHeader.stamp, "base_link");
            printf("Dynamic removal: %f ms\n", removal_time.toc());
            *extractedCloud = *to_be_segmented;
        }
    }

    void inference(pcl::PointCloud<PointType>::Ptr downsampled, pcl::PointCloud<PointType>::Ptr segmented, pcl::VoxelGrid<PointType> filter, size_t nonground_size)
    {
        pcl::PointCloud<PointType>::Ptr query_centroids(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr ref_centroids(new pcl::PointCloud<PointType>());
        pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
        
        std::vector<std::pair<Cluster, Cluster>> closest_pairs;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        std::vector<Cluster> query_clusters = query_map.getMap();
        std::vector<Cluster> ref_clusters = ref_map.getMap();
        std::vector<pcl::PointIndices> query_cluster_indices = query_map.getClusterIndices();
        // Make the closest centroids a pair
        if (query_clusters.size() > 0 && ref_clusters.size() > 0)
        {
            for (auto &ref: ref_clusters)
            {
                PointType pnt;
                pnt.x = ref.centroid_x;
                pnt.y = ref.centroid_y;
                pnt.z = ref.centroid_z;
                ref_centroids->points.push_back(pnt);
            }
            kdtree->setInputCloud(ref_centroids);

            // float dist_sq_threshold = pow(sin(bin_res / 180.0 * M_PI) * 5.0, 2);
            for (auto &query: query_clusters)
            {
                PointType pnt;
                pnt.x = query.centroid_x;
                pnt.y = query.centroid_y;
                pnt.z = query.centroid_z;
                kdtree->nearestKSearch(pnt, 1, pointSearchInd, pointSearchSqDis);
                if (pointSearchInd.size() == 1)
                {
                    Cluster closest_cluster = ref_clusters[pointSearchInd[0]];
                    closest_pairs.push_back(std::make_pair(query, closest_cluster));
                }
            }
        }

        // Compute normal distance
        pointSearchInd.clear();
        pointSearchSqDis.clear();
        pcl::KdTreeFLANN<PointType>::Ptr kdtree2(new pcl::KdTreeFLANN<PointType>());
        std::vector<int> pointSearchInd2;
        std::vector<float> pointSearchSqDis2;

        for (size_t k = 0; k < closest_pairs.size(); ++k)
        {
            Cluster query = closest_pairs[k].first;
            Cluster ref = closest_pairs[k].second;
            pcl::PointCloud<PointType>::Ptr copy_query_ptr(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr copy_ref_ptr(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(query.cloud, *copy_query_ptr);
            pcl::copyPointCloud(ref.cloud, *copy_ref_ptr);
            kdtree->setInputCloud(copy_query_ptr);
            kdtree2->setInputCloud(copy_ref_ptr);
            float errors = 0.0;
            
            Eigen::Vector3f D(query.centroid_x - ref.centroid_x, query.centroid_y - ref.centroid_y, query.centroid_z - ref.centroid_z);

            if (query.cloud.points.size() < 4)
                continue;

            // printf("------Cluster %d\n", query.id);
            for (size_t i = 0; i < query.cloud.points.size(); ++i)
            {
                kdtree->nearestKSearch(query.cloud.points[i], 4, pointSearchInd, pointSearchSqDis);
                kdtree2->nearestKSearch(query.cloud.points[i], 1, pointSearchInd2, pointSearchSqDis2);
                if (pointSearchInd.size() == 4)
                {
                    Eigen::Vector3f A(query.cloud.points[pointSearchInd[1]].x,
                                      query.cloud.points[pointSearchInd[1]].y,
                                      query.cloud.points[pointSearchInd[1]].z);

                    Eigen::Vector3f B(query.cloud.points[pointSearchInd[2]].x,
                                      query.cloud.points[pointSearchInd[2]].y,
                                      query.cloud.points[pointSearchInd[2]].z);
                    
                    Eigen::Vector3f C(query.cloud.points[pointSearchInd[3]].x,
                                      query.cloud.points[pointSearchInd[3]].y,
                                      query.cloud.points[pointSearchInd[3]].z);

                    Eigen::Vector3f X(query.cloud.points[i].x,
                                      query.cloud.points[i].y,
                                      query.cloud.points[i].z);

                    Eigen::Vector3f V(ref.cloud.points[pointSearchInd2[0]].x,
                                      ref.cloud.points[pointSearchInd2[0]].y,
                                      ref.cloud.points[pointSearchInd2[0]].z);

                    Eigen::Vector3f N = (A - B).cross(B - C);

                    float mean_x = (A(0) + B(0) + C(0)) / 3.0;
                    float mean_y = (A(1) + B(1) + C(1)) / 3.0;
                    float mean_z = (A(2) + B(2) + C(2)) / 3.0;

                    // check for normal and displacement vector
                    Eigen::Vector3f N_norm = N.normalized();
                    Eigen::Vector3f D_norm = D.normalized();
                    // float cosine_dist = N_norm(0) * D_norm(0) + N_norm(1) * D_norm(1) + N_norm(2) * D_norm(2);

                    float error = abs(N(0)*(V(0)-mean_x) + 
                                    N(1)*(V(1)-mean_y) + 
                                    N(2)*(V(2)-mean_z)) /
                                    N.norm(); 

                    // float range = sqrt(curr_seg.cloud.points[i].x * curr_seg.cloud.points[i].x +
                    //                    curr_seg.cloud.points[i].y * curr_seg.cloud.points[i].y +
                    //                    curr_seg.cloud.points[i].z * curr_seg.cloud.points[i].z);
                    // V.normalize();
                    
                    // float s = (N(0) * A(0) + N(1) * A(1) + N(2) * A(2)) 
                    //         / (N(0) * V(0) + N(1) * V(1) + N(2) * V(2));
                    // printf("mean: %f; %f; %f; closest: %f; %f; %f; error: %f\n", mean_x, mean_y, mean_z, V(0), V(1), V(2), error);
                    // float error = abs(range - s);
                    errors += error;
                }
            }
            errors /= 1.0 * query.cloud.points.size();

            // temp
            query_map.setFeature(query.id, errors);

            // printf("Pair: %d, %d, error: %f\n", query.id, ref.id, errors);

            // find clusters and segment
            std::vector<int> indices = query_cluster_indices.at(query.id).indices;
            for (size_t j = 0u; j < indices.size(); ++j)
            {
                if (errors != 0.0 && errors < errorThres)
                    downsampled->points[indices[j]].intensity = 1.0; // static
            }
        }

        // preserve points with static label
        pcl::PointCloud<PointType>::Ptr temp_segmented(new pcl::PointCloud<PointType>());
        temp_segmented->reserve(segmented->points.size());
        for (size_t j = 0u; j < segmented->points.size(); ++j)
        {   
            PointType p = segmented->points[j];
            if (j < nonground_size) // nonground
            {
                int voxel_index = filter.getCentroidIndex(p);
                if (voxel_index < 0)
                    continue;
                float label = downsampled->points[voxel_index].intensity;
                if (label == 1.0)
                    temp_segmented->points.push_back(p);
            } 
            else // ground
            {
                temp_segmented->points.push_back(p);
            }

        }
        printf("Downsampled: %d, Original: %d, After-removal: %d\n", (int)downsampled->points.size(), (int)segmented->points.size(), (int)temp_segmented->points.size());
        *segmented = *temp_segmented;
    }

    void publishBoundingBox(ros::Publisher *thisPub, ClusterMap map, ros::Time timestamp, string this_frame)
    {
        if (thisPub->getNumSubscribers() != 0)
        {
            jsk_recognition_msgs::BoundingBoxArray bbox_array;
            bbox_array.header.stamp = timestamp;
            bbox_array.header.frame_id = this_frame;
            std::vector<Cluster> cluster_map = map.getMap();
            for (size_t i = 0; i < cluster_map.size(); ++i)
            {
                jsk_recognition_msgs::BoundingBox bbox = cluster_map[i].bbox;
                if (bbox.value >= 0)
                {
                    bbox.header.stamp = timestamp;
                    bbox.header.frame_id = this_frame;
                    bbox_array.boxes.push_back(bbox);  
                }
            }
            pub_bbox.publish(bbox_array);
        }
    }

    void publishClusteredCloud(ros::Publisher *thisPubCloud, ros::Publisher *thisPubCentroid, ClusterMap map, ros::Time thisStamp, string thisFrame)
    {
        pcl::PointCloud<PointType>::Ptr outPcl (new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr centroids (new pcl::PointCloud<PointType>);
        sensor_msgs::PointCloud2 segmentedCloud;
        sensor_msgs::PointCloud2 centroidCloud;

        visualization_msgs::MarkerArray ids;
        std::vector<Cluster> cluster_map = map.getMap();

        TicToc tic_toc;
        printf("# of clusters: %d\n", cluster_map.size());
        for (int i = 0; i < (int) cluster_map.size(); ++i)
        {
            Cluster cluster = cluster_map[i];
            *outPcl += cluster.cloud;
            PointType centroid;
            centroid.x = cluster.centroid_x;
            centroid.y = cluster.centroid_y;
            centroid.z = cluster.centroid_z;
            centroid.intensity = 100.0;
            centroids->points.emplace_back(centroid);
            
            std::ostringstream stream;
            stream.precision(5);
            stream << cluster.id << ": " << cluster.feature;
            std::string new_string = stream.str();

            visualization_msgs::Marker text;
            text.header.frame_id = thisFrame;
            text.scale.z = 0.5;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 1.0;
            text.color.a = 1.0;
            text.action = 0;
            text.type = 9; // TEXT_VIEW_FACING
            text.id = cluster.id;
            text.text = new_string;
            text.pose.position.x = centroid.x;
            text.pose.position.y = centroid.y;
            text.pose.position.z = centroid.z;
            text.pose.orientation.w = 1.0;
            ids.markers.push_back(text);
        }
        printf("Publish %f ms\n", tic_toc.toc());
    

        if (map.getType() == QUERY)
        {
            if (pub_query_id.getNumSubscribers() != 0)
                pub_query_id.publish(ids);
        }
        else
        {
            if (pub_ref_id.getNumSubscribers() != 0)
                pub_ref_id.publish(ids);
        }

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

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "base_link");
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Lidar Cloud Deskew Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}