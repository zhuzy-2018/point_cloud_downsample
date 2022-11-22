#ifndef _RSLIDAR_DS_COMMON_H_
#define _RSLIDAR_DS_COMMON_H_

#define PCL_NO_PRECOMPILE 
/**Starting with PCL-1.7 you need to define PCL_NO_PRECOMPILE 
 * before you include any PCL headers to include the templated algorithms as well.**/
#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/console/time.h>

#define RS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
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

#endif //_RSLIDAR_DS_COMMON_H_