#include "ekf_slam_3d_cpp/feature_extraction.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

void FeatureExtractor::feature_extraction_gazebo(const pcl::PointCloud<pcl::PointXYZ>& cloud, Eigen::Vector3d& landmark) {

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc_cluster;
    euc_cluster.setClusterTolerance(0.5); /// 50 cm for the cluster size later may need to change it.
    euc_cluster.setMinClusterSize(20);
    euc_cluster.setMaxClusterSize(1000);
    euc_cluster.setSearchMethod(tree);
    euc_cluster.setInputCloud(cloud.makeShared());
    euc_cluster.extract(cluster_indices);

    if (!cluster_indices.empty()) {

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, cluster_indices[0], centroid);
        landmark = Eigen::Vector3d (centroid[0], centroid[1], centroid[2]);

    }


}         

void FeatureExtractor::feature_extraction_multiple_gazebo(const pcl::PointCloud<pcl::PointXYZ>& cloud,
     std::vector<Eigen::Vector3d> &landmarks)  {
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euc_cluster;
    euc_cluster.setClusterTolerance(0.2); /// 30 cm for the cluster size later may need to change it.
    euc_cluster.setMinClusterSize(30);
    euc_cluster.setMaxClusterSize(300);
    euc_cluster.setSearchMethod(tree);
    euc_cluster.setInputCloud(cloud.makeShared());
    euc_cluster.extract(cluster_indices);

    for (const auto &cluster : cluster_indices) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cloud, cluster, centroid);

        landmarks.push_back(Eigen::Vector3d(centroid[0], centroid[1], centroid[2]));
    }

     }