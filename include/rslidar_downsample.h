#ifndef _RSLIDAR_DOWNSAMPLE_H_
#define _RSLIDAR_DOWNSAMPLE_H_

#include <rslidar_ds_common.h>

/**
 * @brief pcl filter factory base
 * 
 */
template<class PointT>
class PCL_FILTER_FACTORY{
public:
    virtual std::shared_ptr<pcl::Filter<PointT>> create_filter() = 0;
    virtual ~PCL_FILTER_FACTORY() = 0;
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

    FilterType _filter_type;

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

    std::shared_ptr<PCL_FILTER_FACTORY<PointT>> filter_factory;
    std::shared_ptr<pcl::Filter<PointT>> _filter;//run time Polymorphism

    unsigned int _random_sample_point;// RANDOM_SAMPLE param
    double _search_radius;//UNIFORM_SAMPLE param
    float _leaf_size;//VOXEL_GRID param
    std::string _field_name;//PASS_THROUGH param
    double _field_limit_min, _field_limit_max;//PASS_THROUGH param
    
    template<typename Param>
    RSLIDAR_DS(FilterType type, const Param& param){//尽力了，只能想到用模板这种编译时多态方法。

        _filter_type = type;

        // _filter = filter_factory->create_filter();

        switch (type)
        {
        case RANDOM_SAMPLE:
            _filter.reset(new pcl::RandomSample<PointT>());
            _random_sample_point = (unsigned int) param;
            _filter->setSample(_random_sample_point);
            break;

        case UNIFORM_SAMPLE:
            _filter.reset(new pcl::UniformSampling<PointT>());
            _search_radius = (float) param;
            _filter->setRadiusSearch(_search_radius);
            break;

        case VOXEL_GRID:
            _filter.reset(new pcl::VoxelGrid<PointT>());
            _leaf_size = (float) param;
            _filter->setLeafSize(_leaf_size, _leaf_size, _leaf_size);
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
        if(type != FilterType::PASS_THROUGH){
            RS_ERROR << "input param *FilterType* wrong , check your parameter !" << RS_ENDL;
            throw std::invalid_argument("input param *FilterType* wrong , check your parameter !");
        }

        _filter_type = type;


        _field_name = field_name;
        _field_limit_min = limit_min;
        _field_limit_max = limit_max;

        _filter.reset(new pcl::PassThrough<PointT>());
        _filter->setFilterFieldName(_field_name);
        _filter->setFilterLimits(_field_limit_min, _field_limit_max);
    }


    // template<typename PointT>
    void set_custom_filter(std::shared_ptr<pcl::Filter<PointT>> custom_filter)
    {
        _filter = custom_filter;
        _filter_type = FilterType::CUSTOM;
    }

    // template<typename PointT>
    void setInputCloud(const pcl::PointCloud<PointT>::ConstPtr& cloud){
        _filter->setInputCloud(cloud);
    }

    // template <typename PointT>
    void filter(pcl::PointCloud<PointT>& cloud){
        _filter->filter(cloud);
    }


};

#endif // _RSLIDAR_DOWNSAMPLE_H_