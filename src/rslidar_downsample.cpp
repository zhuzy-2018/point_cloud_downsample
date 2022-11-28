#include <rslidar_self_car_point_remove_filter.h>
#include <rslidar_downsample.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "rslidar_ds");
    // RSLIDAR_DS<PointXYZIRT> rsds(FilterType::UNIFORM_SAMPLE, 50000);
    RSLIDAR_DS<PointXYZIRT> rsds;


    ros::NodeHandle n = *rsds.get_NodeHandle();
    float x_min, x_max, y_min, y_max, z_min, z_max; 

    n.param<float>("rslidar_ds/self_car_x_min",x_min, -2.5);
    n.param<float>("rslidar_ds/self_car_x_max",x_max, -0.2);
    n.param<float>("rslidar_ds/self_car_y_min",y_min, -0.7);
    n.param<float>("rslidar_ds/self_car_y_max",y_max,  0.7);
    n.param<float>("rslidar_ds/self_car_z_min",z_min, -2  );
    n.param<float>("rslidar_ds/self_car_z_max",z_max,  0.0);

    std::shared_ptr<pcl::Filter<PointXYZIRT>> filter_ptr;
    filter_ptr.reset(new SelfCarRemoveFilter<PointXYZIRT>(x_min, x_max, y_min, y_max, z_min, z_max));
    rsds.set_custom_filter(filter_ptr);
    ros::spin();

    return 0;
    
}