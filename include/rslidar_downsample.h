#ifndef _RSLIDAR_DOWNSAMPLE_H_
#define _RSLIDAR_DOWNSAMPLE_H_

#include <rslidar_ds_common.h>
#include <XmlRpc.h>

/**
 * @brief pcl filter factory base
 * 
 */
template<class PointT>
class PCL_FILTER_FACTORY{
public:
    std::shared_ptr<pcl::Filter<PointT>> _filter;
    PCL_FILTER_FACTORY(){}
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(){return _filter;}
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(double){return _filter;}
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(float){return _filter;}
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(unsigned int){return _filter;}
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(const std::string&, const float&, const float&){return _filter;}
    virtual ~PCL_FILTER_FACTORY(){}
};

template<class PointT>
class PCL_UNIFORM_SAMPLE_FACTORY : public PCL_FILTER_FACTORY<PointT>{
    /**
     * @brief Create a filter object
     * 
     * @param search_radius the 3D grid leaf size
     * @return ** std::shared_ptr<pcl::Filter<PointT>>
     */
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(double search_radius){
        std::shared_ptr<pcl::UniformSampling<PointT>> new_filter(new pcl::UniformSampling<PointT>());
        new_filter->setRadiusSearch(search_radius);
        return new_filter;
    }
};

template<class PointT>
class PCL_RANDOM_SAMPLE_FACTORY : public PCL_FILTER_FACTORY<PointT>{
    /**
     * @brief Create a filter object
     * 
     * @param search_radius Set number of indices to be sampled.
     * @return ** std::shared_ptr<pcl::Filter<PointT>> 
     */
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(unsigned int sample){
        std::shared_ptr<pcl::RandomSample<PointT>> new_filter(new pcl::RandomSample<PointT>());
        new_filter->setSample((unsigned int) sample);
        return new_filter;
    }
};

template<class PointT>
class PCL_VOXEL_GRID_FACTORY : public PCL_FILTER_FACTORY<PointT>{
    /**
     * @brief Create a filter object
     * 
     * @param leaf_size Set the voxel grid leaf size.
     * @return ** std::shared_ptr<pcl::Filter<PointT>> 
     */
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter(float leaf_size){
        std::shared_ptr<pcl::VoxelGrid<PointT>> new_filter(new pcl::VoxelGrid<PointT>());
        new_filter->setLeafSize(leaf_size, leaf_size, leaf_size);
        return new_filter;
    }
};

template<class PointT>
class PCL_PASS_THROUGH_FACTORY : public PCL_FILTER_FACTORY<PointT>{
    /**
     * @brief Create a filter object
     * 
     * @param field_name Provide the name of the field to be used for filtering data.
     * @param limit_min Set the field filter limits.
                        All points having field values outside this interval will be discarded.
     * @param limit_max Set the field filter limits.
                        All points having field values outside this interval will be discarded.
     * @return ** std::shared_ptr<pcl::Filter<PointT>> 
     */
    virtual std::shared_ptr<pcl::Filter<PointT>>
    create_filter(const std::string& field_name , const float& limit_min, const float& limit_max){
        std::shared_ptr<pcl::PassThrough<PointT>> new_filter(new pcl::PassThrough<PointT>());
        new_filter->setFilterFieldName(field_name);
        new_filter->setFilterLimits(limit_min, limit_max);
        return new_filter;
    }
};


enum FilterType{
    RANDOM_SAMPLE,
    UNIFORM_SAMPLE,
    VOXEL_GRID,
    PASS_THROUGH,
    CUSTOM,
    FILTER_NUM
};

template <typename PointT>
class RSLIDAR_DS{
public:

    using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    using PointCloudConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

    FilterType _filter_type;

    ros::NodeHandle nh;
    // XmlRpc::XmlRpcValue filter_params_list;
    std::string topic_in;
    std::string topic_out;
    std::string downsample_mode;
    std::map<std::string, FilterType> string_convertor;


    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    PointCloudPtr cloud_in;
    PointCloudPtr cloud_out;

    std::shared_ptr<PCL_FILTER_FACTORY<PointT>> filter_factory;
    std::shared_ptr<pcl::Filter<PointT>> _filter;//run time Polymorphism
    // std::vector<std::shared_ptr<pcl::Filter<PointT>>> _filter_vec;

    unsigned int _random_sample_point;// RANDOM_SAMPLE param
    double _search_radius;//UNIFORM_SAMPLE param
    float _leaf_size;//VOXEL_GRID param
    std::string _field_name;//PASS_THROUGH param
    double _field_limit_min, _field_limit_max;//PASS_THROUGH param

    void nh_get_param(){
        nh.param<std::string>("rslidar_ds/topic_in", topic_in, "topic_in");
        nh.param<std::string>("rslidar_ds/topic_out",topic_out, "topic_out");
        nh.param<std::string>("rslidar_ds/downsample_mode", downsample_mode, "UNIFORM_SAMPLE");
        int temp_param;
        nh.param<int>("rslidar_ds/random_sample_point",temp_param, 15000);
        _random_sample_point = (unsigned int) temp_param;
        nh.param<double>("rslidar_ds/uniform_sample_search_radius", _search_radius, 0.2);
        nh.param<float>("rslidar_ds/voxel_grid_leaf_size", 0.2);
        nh.param<std::string>("rslidar_ds/pass_through_field_name", _field_name, "z");
        nh.param<double>("rslidar_ds/pass_through_limit_min", _field_limit_min, 0.15);
        nh.param<double>("rslidar_ds/pass_through_limit_max", _field_limit_max, 200);

        // nh.param("rslidar_ds/filters_list", filter_params_list);
        // RS_COUTG << filter_params_list << RS_ENDL;
    }

    void initialization(){
        string_convertor["RANDOM_SAMPLE"] = RANDOM_SAMPLE;
        string_convertor["UNIFORM_SAMPLE"] = UNIFORM_SAMPLE;
        string_convertor["VOXEL_GRID"] = VOXEL_GRID;
        string_convertor["PASS_THROUGH"] = PASS_THROUGH;
        string_convertor["CUSTOM"] = CUSTOM;
        string_convertor["FILTER_NUM"] = FILTER_NUM;

        cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(topic_in, 5, &RSLIDAR_DS::ros_point_cloud_callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_out, 5);
        cloud_in.reset(new pcl::PointCloud<PointT>());
        cloud_out.reset(new pcl::PointCloud<PointT>());
    }
    
    template<typename Param>
    RSLIDAR_DS(FilterType type, const Param& param){//尽力了，只能想到用模板这种编译时多态方法。

        nh_get_param();
        initialization();
        _filter_type = type;

        // _filter = filter_factory->create_filter();

        switch (type)
        {
        case RANDOM_SAMPLE:
            _random_sample_point = (unsigned int) param;
            filter_factory.reset(new PCL_RANDOM_SAMPLE_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_random_sample_point);
            RS_COUTG << "random sample filter --> sample points : " << _random_sample_point << RS_ENDL;
            break;

        case UNIFORM_SAMPLE:
            _search_radius = (float) param;
            filter_factory.reset(new PCL_UNIFORM_SAMPLE_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_search_radius);
            RS_COUTG << "uniform sample --> search radius : " << _search_radius << RS_ENDL;
            break;

        case VOXEL_GRID:
            _leaf_size = (float) param;
            filter_factory.reset(new PCL_VOXEL_GRID_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_leaf_size);
            RS_COUTG << "voxel grid --> leaf size : " << _leaf_size << RS_ENDL;
            break;

        case PASS_THROUGH://交给构造函数重载了
            RS_ERROR << "input param wrong , check your parameter !" << RS_ENDL;
            throw std::invalid_argument("input param wrong , check your parameter !");
            break;
        case CUSTOM:
            RS_COUTG << "use custom filter please remember use set_custom_filter function" << RS_ENDL;
            break;
        
        default:

            break;
        }

    }

    RSLIDAR_DS(FilterType type, const std::string& field_name , const float& limit_min, const float& limit_max){
        nh_get_param();
        initialization();
        if(type != FilterType::PASS_THROUGH){
            RS_ERROR << "input param *FilterType* wrong , check your parameter !" << RS_ENDL;
            throw std::invalid_argument("input param *FilterType* wrong , check your parameter !");
        }

        _filter_type = type;


        _field_name = field_name;
        _field_limit_min = limit_min;
        _field_limit_max = limit_max;

        filter_factory.reset(new PCL_PASS_THROUGH_FACTORY<PointT>());
        _filter = filter_factory->create_filter(_field_name, _field_limit_min, _field_limit_max);
        RS_COUTG << "pass through filter --> field name | field min | field max : " << _field_name
        << " | " << _field_limit_min << " | " << _field_limit_max << RS_ENDL;
    }

    RSLIDAR_DS(){
        nh_get_param();
        initialization();
        switch (string_convertor[downsample_mode])
        {
        case RANDOM_SAMPLE:
            _filter_type = FilterType::RANDOM_SAMPLE;
            filter_factory.reset(new PCL_RANDOM_SAMPLE_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_random_sample_point);
            RS_COUTG << "random sample filter --> sample points : " << _random_sample_point << RS_ENDL;
            break;

        case UNIFORM_SAMPLE:
            _filter_type = FilterType::UNIFORM_SAMPLE;
            filter_factory.reset(new PCL_UNIFORM_SAMPLE_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_search_radius);
            RS_COUTG << "uniform sample --> search radius : " << _search_radius << RS_ENDL;
            break;
        
        case VOXEL_GRID:
            _filter_type = FilterType::VOXEL_GRID;
            filter_factory.reset(new PCL_VOXEL_GRID_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_leaf_size);
            RS_COUTG << "voxel grid --> leaf size : " << _leaf_size << RS_ENDL;
            break;

        case PASS_THROUGH:
            _filter_type = FilterType::PASS_THROUGH;
            filter_factory.reset(new PCL_PASS_THROUGH_FACTORY<PointT>());
            _filter = filter_factory->create_filter(_field_name, _field_limit_min, _field_limit_max);
            RS_COUTG << "pass through filter --> field name | field min | field max : " << _field_name
            << " | " << _field_limit_min << " | " << _field_limit_max << RS_ENDL;
            break;

        case CUSTOM:
            RS_COUTG << "use custom filter please remember use set_custom_filter function" << RS_ENDL;
            break;
            
        
        default:
            RS_ERROR << downsample_mode << " is invalid filter type!! please check ros param!!" << RS_ENDL;
            break;
        }

    }


    // template<typename PointT>
    void set_custom_filter(std::shared_ptr<pcl::Filter<PointT>> custom_filter)
    {
        _filter = custom_filter;
        _filter_type = FilterType::CUSTOM;
    }

    // template<typename PointT>
    void setInputCloud(const PointCloudConstPtr& cloud){
        _filter->setInputCloud(cloud);
    }

    // template <typename PointT>
    void filter(pcl::PointCloud<PointT>& cloud){
        _filter->filter(cloud);
    }

    void ros_point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg_in){
        //input cloud   
        pcl::fromROSMsg(*msg_in, *cloud_in);

        cloud_out = point_cloud_handler(cloud_in);

        //output cloud
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*cloud_out, msg_out);
        msg_out.header = msg_in->header;

        //publish cloud
        cloud_pub.publish(msg_out);
    }

    PointCloudPtr point_cloud_handler(const PointCloudConstPtr& cloud_input){
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_input, indices);

        std::cout << "pointcloud IN points num : " << cloud_input->size() << std::endl;

        pcl::console::TicToc timer;
        timer.tic();

        setInputCloud(cloud_input);

        filter(*cloud_out);

        std::cout << "random downsample time : " << timer.toc() / 1000 << " s "<< std::endl;

        std::cout << "pointcloud OUT points num : " << cloud_out->size() << std::endl;
        
        return cloud_out;
    }


};

#endif // _RSLIDAR_DOWNSAMPLE_H_