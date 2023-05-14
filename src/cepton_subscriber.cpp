#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "cepton2_ros/point.hpp"
#include "cepton_sdk2.h"

void cloudCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ceptonPointCloud)
{
    auto& points = ceptonPointCloud->points;
    if (points.size() == 0) return;
    PCL_INFO("Received %i points at timestamp: %i.\n", (int)points.size(), ceptonPointCloud->header.stamp);
    PCL_INFO("First point:\tx: %f, y: %f, z: %f.\n", points[0].x, points[0].y, points[0].z);
    PCL_INFO("First point:\tx: %f, y: %f, z: %f.\n", points[points.size()-1].x, points[points.size()-1].y, points[points.size()-1].z);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "cepton_subscriber");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZI>>("cepton2/points", 10, cloudCallback);
    
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;    
}