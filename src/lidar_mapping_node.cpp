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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar_mapping_node.hpp"
#include "floam/lidar_mapping.hpp"
#include "floam/lidar_imager.hpp"
#include "floam/lidar_scanner.hpp"

namespace floam
{
namespace lidar
{

LidarMappingNode::LidarMappingNode()
{
  // constructor
}

LidarMappingNode::~LidarMappingNode()
{
  // destructor
}

void LidarMappingNode::onInit()
{
  m_nodeHandle = getPrivateNodeHandle();

  int scan_line = 64;
  double vertical_angle = 2.0;
  double scan_period= 0.1;
  double max_dis = 60.0;
  double min_dis = 2.0;
  double map_resolution = 0.4;

  m_nodeHandle.getParam("scan_period", scan_period); 
  m_nodeHandle.getParam("vertical_angle", vertical_angle); 
  m_nodeHandle.getParam("max_dis", max_dis);
  m_nodeHandle.getParam("min_dis", min_dis);
  m_nodeHandle.getParam("scan_line", scan_line);
  m_nodeHandle.getParam("map_resolution", map_resolution);
  m_lidar.setScanPeriod(scan_period);
  m_lidar.setVerticalAngle(vertical_angle);
  m_lidar.setLines(scan_line);
  m_lidar.setMaxDistance(max_dis);
  m_lidar.setMinDistance(min_dis);
  
  m_lidarMapping.init(map_resolution);

  m_subPoints = m_nodeHandle.subscribe<sensor_msgs::PointCloud2>("points_filtered", 100, &LidarMappingNode::handlePoints, this);
  m_subOdom = m_nodeHandle.subscribe<nav_msgs::Odometry>("odom", 100, &LidarMappingNode::handleOdom, this);
  
  m_pubMap = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/map", 100);
  // std::thread lidar_mapping_process{floam::lidar::lidar_mapping};
}

void LidarMappingNode::handleOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    m_mutexLock.lock();
    m_odometry.push(msg);
    m_mutexLock.unlock();
}

void LidarMappingNode::handlePoints(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg)
{
    m_mutexLock.lock();
    m_points.push(lidarCloudMsg);
    m_mutexLock.unlock();
}


void LidarMappingNode::mapping()
{
    while(1){
        if(!m_odometry.empty() && !m_points.empty()){

            //read data
            m_mutexLock.lock();
            if(!m_points.empty() && m_points.front()->header.stamp.toSec()<m_odometry.front()->header.stamp.toSec()-0.5*m_lidar.scan_period){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> lidar mapping node"); 
                m_points.pop();
                m_mutexLock.unlock();
                continue;              
            }

            if(!m_odometry.empty() && m_odometry.front()->header.stamp.toSec() < m_points.front()->header.stamp.toSec()-0.5*m_lidar.scan_period){
                m_odometry.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> lidar mapping node");
                m_mutexLock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*m_points.front(), *pointcloud_in);
            ros::Time pointcloud_time = (m_points.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(m_odometry.front()->pose.pose.orientation.w,m_odometry.front()->pose.pose.orientation.x,m_odometry.front()->pose.pose.orientation.y,m_odometry.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(m_odometry.front()->pose.pose.position.x,m_odometry.front()->pose.pose.position.y,m_odometry.front()->pose.pose.position.z));
            m_points.pop();
            m_odometry.pop();
            m_mutexLock.unlock();
            

            m_lidarMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = m_lidarMapping.getMap();
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*pc_map, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "map";
            m_pubMap.publish(PointsMsg); 
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

}  // namespace lidar
}  // namespace floam


#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::lidar::LidarMappingNode, nodelet::Nodelet);