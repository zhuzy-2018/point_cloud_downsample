#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (uint8_t, intensity, intensity) 
(uint16_t, ring, ring) (double, timestamp, timestamp))

ros::Subscriber cloud_sub;
ros::Publisher cloud_pub;

void pointCloud_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);
    //rewrite intensity
    static double max_time = 0;

    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto  &point = cloud_in->points[i];
        max_time = std::max(max_time, point.timestamp);
    }

    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto  &point = cloud_in->points[i];
        point.intensity = (uint8_t)(point.timestamp * 255 / max_time); 
    }

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_in, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_robosense_cloud");

    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("/rslidar_points", 10, pointCloud_callback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_modified", 10);

    ros::spin();

    return 0;
}