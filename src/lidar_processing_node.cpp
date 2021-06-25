// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar_processing_node.hpp"
#include "floam/lidar.hpp"
#include "floam/lidar_processing.hpp"


namespace floam
{
namespace lidar
{

LidarProcessingNode::LidarProcessingNode()
{
  // constructor
}

LidarProcessingNode::~LidarProcessingNode()
{
  // destructor
}

void LidarProcessingNode::onInit()
{
    m_nodeHandle = getPrivateNodeHandle();

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    m_nodeHandle.getParam("scan_period", scan_period); 
    m_nodeHandle.getParam("vertical_angle", vertical_angle); 
    m_nodeHandle.getParam("max_dis", max_dis);
    m_nodeHandle.getParam("min_dis", min_dis);
    m_nodeHandle.getParam("scan_line", scan_line);

    m_lidar.setScanPeriod(scan_period);
    m_lidar.setVerticalAngle(vertical_angle);
    m_lidar.setLines(scan_line);
    m_lidar.setMaxDistance(max_dis);
    m_lidar.setMinDistance(min_dis);

    m_lidarProcessing.init(m_lidar);

    m_subPoints = m_nodeHandle.subscribe<sensor_msgs::PointCloud2>("points", 100, &LidarProcessingNode::handlePoints, this);

    m_pubPointsFiltered = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("points_filtered", 100);

    m_pubEdgePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("cloud_edge", 100);

    m_pubSurfacePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("cloud_surface", 100); 

    //std::thread lidar_processing_process{floam::lidar::lidar_processing};
}

void LidarProcessingNode::handlePoints(const sensor_msgs::PointCloud2ConstPtr &points)
{
    m_mutexLock.lock();
    m_points.push(points);
    m_mutexLock.unlock();
}

void LidarProcessingNode::lidarProcessing()
{
    while(1){
        if(!m_points.empty()){
            //read data
            m_mutexLock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*m_points.front(), *pointcloud_in);
            ros::Time pointcloud_time = (m_points.front())->header.stamp;
            m_points.pop();
            m_mutexLock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            m_lidarProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average lidar processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 lidarCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, lidarCloudFilteredMsg);
            lidarCloudFilteredMsg.header.stamp = pointcloud_time;
            lidarCloudFilteredMsg.header.frame_id = "base_link";
            m_pubPointsFiltered.publish(lidarCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            m_pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            m_pubSurfacePoints.publish(surfPointsMsg);
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


}  // namespace lidar
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::lidar::LidarProcessingNode, nodelet::Nodelet)