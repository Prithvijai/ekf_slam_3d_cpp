#ifndef SLAM_NODE_HPP
#define SLAM_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/Dense>  // Numpy for C++, for matrix multiplication and manipulations task contains VectorXd and MatrixXd .
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/filters/voxel_grid.h>
#include "ekf_slam_3d_cpp/ekf_slam_core.hpp"

#include "ekf_slam_3d_cpp/feature_extraction.hpp"


// all the powers that a normal ROS 2 Node has (like the ability to talk to topics, use timers, and log messages to your terminal).
class EKFSLAMNode : public rclcpp::Node {
    public:
        EKFSLAMNode();
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ ;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ ;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_ ;

        Eigen::VectorXd state_;  //state vector    // this VectorXd create [a,s f, agare, agreg, ...] vector,
        Eigen::MatrixXd p_ , Q_ ;   //(covariance matrix) and process noise
        // Eigen::MatrixXd Q_ ;   
        rclcpp::Time last_time_ ;  // create timestamp variable that can store the time stamp of last callback occurs 

        std::unique_ptr<EKFSLAMCore> slam_core_;
        std::unique_ptr<FeatureExtractor> feature_;


        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr  msg);
        void point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};  



#endif