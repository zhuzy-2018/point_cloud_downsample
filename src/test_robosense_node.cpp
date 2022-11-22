#define PCL_NO_PRECOMPILE 
/**Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE 
 * before you include any PCL headers to include the templated algorithms as well.**/
#include <iostream>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/console/time.h>

#define RS_COUTG  std::cout << "\033[32m"         // green
#define RS_ENDL    "\033[0m" << std::endl

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) 
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
    static double max_time;
    static double min_time;
    max_time = DBL_MIN;
    min_time = DBL_MAX;

    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto  &point = cloud_in->points[i];
        max_time = std::max(max_time, point.timestamp);
        min_time = std::min(min_time, point.timestamp);
    }

    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto  &point = cloud_in->points[i];
        point.intensity = (uint8_t)((point.timestamp - min_time) * 255 / (max_time - min_time)); 
    }
    ROS_INFO("min | max | max_time_stamp: %f | %f | %f",min_time, max_time, (max_time - min_time));

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_in, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

void XYZIRT_pointCloud_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto point = cloud_in->points[i];

        std::cout<< "X | Y | Z | I | R | T: "<< point.x << " | "<< point.y << " | " << point.z << " | " << point.intensity << " | " << point.ring << " | "<< point.timestamp <<std::endl;
    }

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_in, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

void XYZI_pointCloud_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    pcl::console::TicToc timer;
    timer.tic();
    #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < cloud_in->size(); i++)
    {
        auto point = cloud_in->points[i];

        point.intensity = point.x;
    }

    RS_COUTG << "random downsample time : " << timer.toc() << " ms "<< RS_ENDL;    
    

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_in, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

void pointCloud_randown_sample_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, indices);

    std::cout << "pointcloud IN points num : " << cloud_in->size() << std::endl;

    pcl::console::TicToc timer;
    timer.tic();

    pcl::RandomSample<PointXYZIRT> rs;
    rs.setInputCloud(cloud_in);
    int N_SCAN = 128;
    int Horizon_SCAN = 900;
    float downsample_rate = 0.5;
    rs.setSample((unsigned int)(N_SCAN * Horizon_SCAN * downsample_rate));

    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRT>());
    rs.filter(*cloud_out);

    std::cout << "random downsample time : " << timer.toc() / 1000 << " s "<< std::endl;

    std::cout << "pointcloud OUT points num : " << cloud_out->size() << std::endl;

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

void pointCloud_uniform_sample_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, indices);

    std::cout << "pointcloud IN points num : " << cloud_in->size() << std::endl;

    pcl::console::TicToc timer;
    timer.tic();

    pcl::UniformSampling<PointXYZIRT> us;
    us.setInputCloud(cloud_in);
    int N_SCAN = 128;
    int Horizon_SCAN = 900;
    float downsample_rate = 0.5;
    us.setRadiusSearch(0.2f);

    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRT>());
    us.filter(*cloud_out);

    std::cout << "random downsample time : " << timer.toc() / 1000 << " s "<< std::endl;

    std::cout << "pointcloud OUT points num : " << cloud_out->size() << std::endl;

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rslidar_downsample_cloud");

    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("/lidar/vlp32_middle/PointCloud2_compensated", 10, XYZI_pointCloud_callback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_modified", 10);

    ros::spin();

    return 0;
}