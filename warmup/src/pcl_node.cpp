#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <sstream>

ros::Subscriber pointcloud_sub;
ros::Publisher pointcloud_pub;

void pointcloud_callback(sensor_msgs::PointCloud pointcloud)
{
    sensor_msgs::PointCloud2 pointcloud2;
    sensor_msgs::convertPointCloudToPointCloud2(pointcloud, pointcloud2);
    pointcloud_pub.publish(pointcloud2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle n;
    pointcloud_sub = n.subscribe<sensor_msgs::PointCloud>("/projected_stable_scan", 10, pointcloud_callback);
    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/projected_stable_scan2", 10);
    ros::Rate loop_rate(10000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}