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
#include "floam/lidar.hpp"
#include "floam/lidar_processing.hpp"


namespace floam
{
namespace lidar
{

LidarProcessing lidarProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLidarCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(lidarCloudMsg);
    mutex_lock.unlock();
   
}

double total_time =0;
int total_frame=0;

void lidar_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            lidarProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
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
            pubLidarCloudFiltered.publish(lidarCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


}  // namespace lidar
}  // namespace floam


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    floam::lidar::lidar_param.setScanPeriod(scan_period);
    floam::lidar::lidar_param.setVerticalAngle(vertical_angle);
    floam::lidar::lidar_param.setLines(scan_line);
    floam::lidar::lidar_param.setMaxDistance(max_dis);
    floam::lidar::lidar_param.setMinDistance(min_dis);

    floam::lidar::lidarProcessing.init(floam::lidar::lidar_param);

    ros::Subscriber subLidarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, floam::lidar::velodyneHandler);

    floam::lidar::pubLidarCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    floam::lidar::pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/lidar_cloud_edge", 100);

    floam::lidar::pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/lidar_cloud_surf", 100); 

    std::thread lidar_processing_process{floam::lidar::lidar_processing};

    ros::spin();

    return 0;
}