#ifndef FEATURE_EXTRACTION_CPP
#define FEATURE_EXTRACTION_CPP

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>


class FeatureExtractor {
    public:

    void feature_extraction_gazebo(const pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3d &landmark);


};

#endif