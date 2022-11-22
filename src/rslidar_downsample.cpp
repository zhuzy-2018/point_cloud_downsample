#include <rslidar_downsample.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "rslidar_ds");
    RSLIDAR_DS<PointXYZIRT> rsds(FilterType::UNIFORM_SAMPLE, 50000);

    ros::spin();

    return 0;
    
}