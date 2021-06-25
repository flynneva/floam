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
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/odom_estimation_node.hpp"
#include "floam/lidar.hpp"
#include "floam/odom_estimation.hpp"

namespace floam
{
namespace odom
{

OdomEstimationNode::OdomEstimationNode()
{
  // constructor
}

OdomEstimationNode::~OdomEstimationNode()
{
  // destructor
}

void OdomEstimationNode::onInit()
{
  m_nodeHandle = getPrivateNodeHandle();

  float scan_period, vertical_angle, max_dis, min_dis;
  int scan_line, map_resolution;

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
  m_odomEstimation.init(m_lidar, map_resolution);

  ros::Subscriber subEdgelidarCloud = m_nodeHandle.subscribe<sensor_msgs::PointCloud2>("lidar_cloud_edge", 100, &OdomEstimationNode::edgeHandler, this);
  ros::Subscriber subSurflidarCloud = m_nodeHandle.subscribe<sensor_msgs::PointCloud2>("lidar_cloud_surf", 100, &OdomEstimationNode::surfaceHandler, this);  
  pubLidarOdometry = m_nodeHandle.advertise<nav_msgs::Odometry>("odom", 100);
  // std::thread m_odomEstimation{OdomEstimationNode::odomEstimation};
}

void OdomEstimationNode::surfaceHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg)
{
    m_mutexLock.lock();
    m_pointsSurface.push(lidarCloudMsg);
    m_mutexLock.unlock();
}

void OdomEstimationNode::edgeHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg)
{
    m_mutexLock.lock();
    m_pointsEdge.push(lidarCloudMsg);
    m_mutexLock.unlock();
}

void OdomEstimationNode::odomEstimation()
{
    while(1){
        if(!m_pointsEdge.empty() && !m_pointsSurface.empty()){

            //read data
            m_mutexLock.lock();
            if(!m_pointsSurface.empty() && (m_pointsSurface.front()->header.stamp.toSec()<m_pointsEdge.front()->header.stamp.toSec()-0.5*m_lidar.scan_period)){
                m_pointsSurface.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                m_mutexLock.unlock();
                continue;  
            }

            if(!m_pointsEdge.empty() && (m_pointsEdge.front()->header.stamp.toSec()<m_pointsSurface.front()->header.stamp.toSec()-0.5*m_lidar.scan_period)){
                m_pointsEdge.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                m_mutexLock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*m_pointsEdge.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*m_pointsSurface.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (m_pointsSurface.front())->header.stamp;
            m_pointsEdge.pop();
            m_pointsSurface.pop();
            m_mutexLock.unlock();

            if(is_odom_inited == false){
                m_odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                m_odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);
            }



            Eigen::Quaterniond q_current(m_odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = m_odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry lidarOdometry;
            lidarOdometry.header.frame_id = "map";
            lidarOdometry.child_frame_id = "base_link";
            lidarOdometry.header.stamp = pointcloud_time;
            lidarOdometry.pose.pose.orientation.x = q_current.x();
            lidarOdometry.pose.pose.orientation.y = q_current.y();
            lidarOdometry.pose.pose.orientation.z = q_current.z();
            lidarOdometry.pose.pose.orientation.w = q_current.w();
            lidarOdometry.pose.pose.position.x = t_current.x();
            lidarOdometry.pose.pose.position.y = t_current.y();
            lidarOdometry.pose.pose.position.z = t_current.z();
            pubLidarOdometry.publish(lidarOdometry);
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

}  // namespace odom
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::odom::OdomEstimationNode, nodelet::Nodelet)