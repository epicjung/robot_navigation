#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/tf.h>

namespace mapHandle{
    
    nav_msgs::OccupancyGrid cvimg2occumap(cv::Mat cvimg, float resolution,cv::Point origin_cvpt){
        nav_msgs::OccupancyGrid m_gridmap;
        m_gridmap.info.resolution = resolution;
        geometry_msgs::Pose origin;
        origin.position.x = -origin_cvpt.x * resolution; origin.position.y = -origin_cvpt.y * resolution;
        origin.orientation.w = 1;
        m_gridmap.info.origin = origin;
        m_gridmap.info.width = cvimg.size().width;
        m_gridmap.info.height = cvimg.size().height;
        //ROS_INFO("[CV 2 OCCUGRID CONVERSION] PUT CV DATA");
        for(int i=0;i<m_gridmap.info.width*m_gridmap.info.height;i++) m_gridmap.data.push_back(-1);
        //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID SIZE : %d",m_gridmap.data.size());
        //ROS_INFO("[CV 2 OCCUGRID CONVERSION] GRID ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);

        for(int y = 0; y < cvimg.size().height; y++)
        {
        for(int x = 0; x < cvimg.size().width; x++)
        {
            int tmpdata = cvimg.at<unsigned char>(y,x);
            int ttmpdata = -1; //Unknown
            if(tmpdata >= 150) //free
            {
            ttmpdata = (tmpdata - 250) / -2;
            if(ttmpdata < 0)
                ttmpdata = 0;
            }
            else if(tmpdata <= 98)
            {
            ttmpdata = (tmpdata - 200) / -2;
            if(ttmpdata > 100)
                ttmpdata = 100;
            }
            m_gridmap.data.at(x + m_gridmap.info.width * y) = ttmpdata;
        }
        }
        return m_gridmap;
    }

    cv::Mat occumap2cvimg( nav_msgs::OccupancyGrid occumap){
        // unkown = -1 -> 99~149, free : 0:50 ->250:150, occupied :51:100 -> 0:98
        double resolution = occumap.info.resolution;
        cv::Point origin_cvpt(-occumap.info.origin.position.x / resolution,
                            -occumap.info.origin.position.y / resolution);
        cv::Size img_size;
        cv::Mat cvimg = cv::Mat::zeros( occumap.info.height,occumap.info.width,CV_8UC1);
        //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] GRID SIZE : %d",occumap.data.size());
        //ROS_INFO("[OCCUGRID 2 CVT CONVERSION] CV ORIGIN : [%d,%d]",origin_cvpt.x,origin_cvpt.y);
        for(int pt = 0;pt < occumap.data.size();pt++)
        {
        int pt_y = pt / occumap.info.width;
        int pt_x = pt % occumap.info.width;
        int value = occumap.data.at(pt);
        unsigned char img_value;
        if(value == -1) img_value = 120;
        else if (value <= 50) img_value = 250 - 2 * value;
        else if (value >=51) img_value = 200 - 2 * value;
        cvimg.at<unsigned char>(pt_y,pt_x) = img_value;
        }
        return cvimg;
    }

    void saveOccupanymap(std::string filepath, nav_msgs::OccupancyGrid gridmap_in){
        cv::Mat occumat = occumap2cvimg(gridmap_in);
        std::stringstream strm_png;
        strm_png << filepath << ".png";
        std::stringstream strm_info;
        strm_info << filepath << ".csv";

        cv::imwrite(strm_png.str(),occumat);

        std::ofstream filesave(strm_info.str().c_str());
        if(filesave.is_open())
        {
        filesave << gridmap_in.info.resolution << "\n";
        filesave << gridmap_in.info.origin.position.x << "\n";
        filesave << gridmap_in.info.origin.position.y << "\n";
        }
        filesave.close();
    }

    bool loadOccupancymap(std::string filepath, nav_msgs::OccupancyGrid& gridmap_out){
        std::stringstream strm_png;
        strm_png << filepath <<".png";
        std::stringstream strm_info;
        strm_info << filepath << ".csv";
        cv::Mat occumat = cv::imread(strm_png.str(),cv::IMREAD_GRAYSCALE);
        std::ifstream fileload(strm_info.str().c_str());
        float resolution,origin_x,origin_y;
        std::vector<std::string>   result;
        std::string line;
        if(!fileload.is_open())
        {
        std::cout << "Warning : Canot open occupancy map" << std::endl;
        return false;
        }
        while(std::getline(fileload,line)) result.push_back(line);
        if(result.size()!=3)
        {
        std::cout << "Warning : Canot open occupancy map (arguments)" << std::endl;
        return false;
        }
        resolution = std::atof(result.at(0).c_str());
        origin_x = std::atof(result.at(1).c_str());
        origin_y = std::atof(result.at(2).c_str());
        gridmap_out = cvimg2occumap(occumat, resolution,cv::Point(- origin_x / resolution,-origin_y / resolution));
        gridmap_out.header.frame_id = "map";

        return true;
    }

    Eigen::Matrix4f mat2eigen(cv::Mat mat)
    {
        Eigen::Matrix4f result =  Eigen::Matrix4f::Identity();

        result(0,0) = mat.at<float>(0,0);
        result(0,1) = mat.at<float>(0,1);
        result(0,2) = mat.at<float>(0,2);
        result(0,3) = mat.at<float>(0,3);

        result(1,0) = mat.at<float>(1,0);
        result(1,1) = mat.at<float>(1,1);
        result(1,2) = mat.at<float>(1,2);
        result(1,3) = mat.at<float>(1,3);

        result(2,0) = mat.at<float>(2,0);
        result(2,1) = mat.at<float>(2,1);
        result(2,2) = mat.at<float>(2,2);
        result(2,3) = mat.at<float>(2,3);

        result(3,0) = mat.at<float>(3,0);
        result(3,1) = mat.at<float>(3,1);
        result(3,2) = mat.at<float>(3,2);
        result(3,3) = mat.at<float>(3,3);

        return result;
    }

    cv::Mat eigen2mat(Eigen::Matrix4f mat)
     {
         cv::Mat result = cv::Mat::zeros(4,4,CV_32FC1);
         result.at<float>(0,0) = mat(0,0);
         result.at<float>(0,1) = mat(0,1);
         result.at<float>(0,2) = mat(0,2);
         result.at<float>(0,3) = mat(0,3);

         result.at<float>(1,0) = mat(1,0);
         result.at<float>(1,1) = mat(1,1);
         result.at<float>(1,2) = mat(1,2);
         result.at<float>(1,3) = mat(1,3);

         result.at<float>(2,0) = mat(2,0);
         result.at<float>(2,1) = mat(2,1);
         result.at<float>(2,2) = mat(2,2);
         result.at<float>(2,3) = mat(2,3);

         result.at<float>(3,0) = mat(3,0);
         result.at<float>(3,1) = mat(3,1);
         result.at<float>(3,2) = mat(3,2);
         result.at<float>(3,3) = mat(3,3);

         return result;
     }

     geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose)
    {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double)pose(0,0),
                (double)pose(0,1),
                (double)pose(0,2),
                (double)pose(1,0),
                (double)pose(1,1),
                (double)pose(1,2),
                (double)pose(2,0),
                (double)pose(2,1),
                (double)pose(2,2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0,3);
        geoPose.position.y = pose(1,3);
        geoPose.position.z = pose(2,3);

        return geoPose;
    }

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose)
    {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 m(q);
        result(0,0) = m[0][0];
        result(0,1) = m[0][1];
        result(0,2) = m[0][2];
        result(1,0) = m[1][0];
        result(1,1) = m[1][1];
        result(1,2) = m[1][2];
        result(2,0) = m[2][0];
        result(2,1) = m[2][1];
        result(2,2) = m[2][2];
        result(3,3) = 1;

        result(0,3) = geoPose.position.x;
        result(1,3) = geoPose.position.y;
        result(2,3) = geoPose.position.z;

        return result;
    }
}

