#ifndef _RSLIDAR_SELF_CAR_POINT_REMOVE_FILTER_H_
#define _RSLIDAR_SELF_CAR_POINT_REMOVE_FILTER_H_


#include <rslidar_ds_common.h>
#include <omp.h>

#define RS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define RS_COUTG  std::cout << "\033[32m"         // green
#define RS_ENDL    "\033[0m" << std::endl

template<typename PointT>
class SelfCarRemoveFilter : public pcl::Filter<PointT>{
public:
    using pcl::Filter<PointT>::filter_name_;
    using pcl::Filter<PointT>::getClassName;
    using pcl::Filter<PointT>::input_;

   float x_min, x_max, y_min, y_max, z_min, z_max; 

    SelfCarRemoveFilter(float x_min, float x_max,
                        float y_min, float y_max,
                        float z_min, float z_max):
                        x_min(x_min), x_max(x_max), 
                        y_min(y_min), y_max(y_max),
                        z_min(z_min), z_max(z_max){
                            filter_name_ = "SelfCarRemoveFilter";
                            RS_COUTG << filter_name_ << "-> x_min | x_max | y_min | y_max | z_min | z_max |"
                            << x_min << " | " << x_max << " | " 
                            << y_min << " | " << y_max << " | "
                            << z_min << " | " << z_max << RS_ENDL;
                        }

    inline bool point_in_bbox(PointT pt){
        if((pt.x > x_min) && (pt.x < x_max) &&
           (pt.y > y_min) && (pt.y < y_max) &&
           (pt.z > z_min) && (pt.z < z_max))
            return true;

        else
            return false;
    }

    void applyFilter (pcl::PointCloud<PointT> &output){
        // Has the input dataset been set already?
        if (!input_)
        {
            PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName ().c_str ());
            output.width = output.height = 0;
            output.points.clear ();
            return;
        }

        output.height       = 1;                    // downsampling breaks the organized structure
        output.is_dense     = true;                 // we filter out invalid points

        std::vector<int> indices;

        int count = 0;
        
        for(size_t i = 0; i < input_->points.size(); i++){
            if(!point_in_bbox(input_->points[i])){
                indices.push_back(static_cast<int>(i));
                count++;
            }
        }

        output.points.clear();
        

        for(size_t i = 0; i < indices.size(); i++){
            output.points.push_back(input_->points[indices[i]]);
        }

        output.width = output.points.size();

    }
};
#endif