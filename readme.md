# ros包-点云降采样

本ros包虽然名字叫rslidar_downsample
然而其实可以用于各种点云降采样

本包依赖pcl的滤波器，但是区别于直接使用pcl的滤波器，本包对常用的几种滤波器做了封装。

并且**可以通过读取config/rslidar_ds_param.yaml的<a href="#info"> 参数列表</a>添加一系列的滤波器**

**点云会依次通过这些滤波器**，然后以ros的形式输出

同时RSLIDAR_DS类还可以不依赖ROS，通过调用`point_cloud_handler`函数，可以直接以`pcl::PointCloud`的格式降采样

RSLIDAR_DS还支持使用自定义的滤波器，详见<a href="#info"> 自定义滤波器</a>

## requirement

* pcl >= 1.8.0
* OpenMP

## <a id="info">参数列表</a>
内置pcl滤波器类型为: `RANDOM_SAMPLE`, `UNIFORM_SAMPLE`, `VOXEL_GRID`, `PASS_THROUGH`。

对应参数为：
* `random_sample_point`
* `uniform_sample_search_radius`
* `voxel_grid_leaf_size`
* `pass_through_field_name`
* `pass_through_limit_min`
* `pass_through_limit_max`

参数具体内涵，参考[pcl官方文档](http://pointclouds.org/documentation/group__filters.html)

## <a id="info">自定义滤波器</a>

参考`rslidar_self_car_point_remove_filter.h`文件中的写法

* 自定义的滤波器需要public继承pcl::Filter
* 自定义的滤波器需要重写构造函数，并且最好输出**类型**和**参数**
* 自定义的滤波器需要重写`applyFilter()`函数，这个函数会被`pcl::Filter::filter()`调用，从而返回点云输出
* 调用`setInputCloud()`后，输入的点云变量为`input_`，需要声明
``` c++
using pcl::Filter<PointT>::input_;
```


