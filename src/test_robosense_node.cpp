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

#include <rslidar_ds_common.h>

#include <pcl/console/time.h>




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

void pointCloud_crop_hull_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, indices);

    std::cout << "pointcloud IN points num : " << cloud_in->size() << std::endl;

    pcl::PointCloud<PointXYZIRT>::Ptr bbox(new pcl::PointCloud<PointXYZIRT>());
    PointXYZIRT p1, p2, p3, p4, p5, p6, p7, p8;
    float x_min, x_max, y_min, y_max, z_min, z_max;

    x_min = -2.5; x_max = -0.2; y_min = -0.7; y_max = 0.7; z_min = -1; z_max = -0.2;

    p1.x = x_min; p1.y = y_min; p1.z = z_min; bbox->push_back(p1);
    p2.x = x_min; p2.y = y_min; p2.z = z_max; bbox->push_back(p2);
    p3.x = x_min; p3.y = y_max; p3.z = z_min; bbox->push_back(p3);
    p4.x = x_min; p4.y = y_max; p4.z = z_max; bbox->push_back(p4);
    p5.x = x_max; p5.y = y_min; p5.z = z_min; bbox->push_back(p5);
    p6.x = x_max; p6.y = y_min; p6.z = z_max; bbox->push_back(p6);
    p7.x = x_max; p7.y = y_max; p7.z = z_min; bbox->push_back(p7);
    p8.x = x_max; p8.y = y_max; p8.z = z_max; bbox->push_back(p8);

    pcl::ConvexHull<PointXYZIRT> hull;
    hull.setInputCloud(bbox);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<PointXYZIRT>::Ptr polygons_points(new pcl::PointCloud<PointXYZIRT>());
    hull.reconstruct(*polygons_points, polygons);


    

    pcl::console::TicToc timer;
    timer.tic();

    pcl::CropHull<PointXYZIRT> ch;
    ch.setDim(3);
    ch.setInputCloud(cloud_in); 
    ch.setHullIndices(polygons);
    ch.setHullCloud(polygons_points);


    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRT>());
    ch.filter(*cloud_out);


    std::cout << "random downsample time : " << timer.toc() / 1000 << " s "<< std::endl;

    std::cout << "pointcloud OUT points num : " << cloud_out->size() << std::endl;

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

template <typename PointT>
bool point_in_bbox(PointT& pt){
    float x_min, x_max, y_min, y_max, z_min, z_max;

    x_min = -2.5; x_max = -0.2; y_min = -0.7; y_max = 0.7; z_min = -2; z_max = 0;

    if((pt.x > x_min) && (pt.x < x_max) &&
       (pt.y > y_min) && (pt.y < y_max))
        return true;

    else
        return false;
}

template <typename PointT>
bool point_ring_under(PointT& pt, size_t ring){
    if (pt.ring <= ring)
        return true;

    else
        return false;
}

void pointCloud_crop_bbox_callback(const sensor_msgs::PointCloud2Ptr& msg_in)
{
    //input cloud   
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_in;
    cloud_in.reset(new pcl::PointCloud<PointXYZIRT>());
    static pcl::PointCloud<PointXYZIRT>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRT>());
    pcl::moveFromROSMsg(*msg_in, *cloud_in);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, indices);

    std::cout << "pointcloud IN points num : " << cloud_in->size() << std::endl;


    // float x_min, x_max, y_min, y_max, z_min, z_max;

    // x_min = -2.5; x_max = -0.2; y_min = -0.7; y_max = 0.7; z_min = -1; z_max = -0.2;

    float ptx, pty, ptz;
    ptx = -0.977; pty = 0.00366; ptz = -0.343;
    PointXYZIRT temp_pt;
    temp_pt.x = ptx; temp_pt.y = pty; temp_pt.z = ptz;
    RS_COUTG << "is temp point in:" << point_in_bbox(temp_pt) << RS_ENDL;

    auto iter = cloud_in->points.begin();
    pcl::console::TicToc timer;
    timer.tic();
    // #pragma omp parallel for num_threads(4)
    for(size_t i = 0; i < cloud_in->size(); i++){
        if(point_in_bbox(*iter))
            cloud_in->points.erase(iter);
        else
            iter++;
    }
    int count = 0;
    int count2 = 0;

    // for(auto iter2 = cloud_in->points.begin(); iter2 != cloud_in->points.end();){
    //     if(point_in_bbox(*iter2)){
    //         cloud_in->points.erase(iter2);
    //         count++;
    //     }
    //     else
    //         iter2++;
    //     count2++;
    // }
    // RS_COUTG << "in bbox points: " << count << RS_ENDL;
    // RS_COUTG << "iterations times: " << count2 << RS_ENDL;

    
    cloud_in->width = cloud_in->size();
    cloud_in->height = 1;


    std::cout << "random downsample time : " << timer.toc() / 1000 << " s "<< std::endl;

    std::cout << "pointcloud OUT points num : " << cloud_in->size() << std::endl;

    //output cloud
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_in, msg_out);
    msg_out.header = msg_in->header;

    //publish cloud
    cloud_pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rslidar_downsample_cloud");

    ros::NodeHandle nh;

    cloud_sub = nh.subscribe("/lidar/vlp32_middle/PointCloud2_compensated", 10, pointCloud_crop_bbox_callback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points_modified", 10);

    ros::spin();

    return 0;
}