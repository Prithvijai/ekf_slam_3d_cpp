#include "rclcpp/rclcpp.hpp"
#include "ekf_slam_3d_cpp/ekf_slam_core.hpp"

void EKFSLAMCore::predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P, double dt,
                             double r_v, double p_v, double y_v,
                             double ax, double ay, double az) {

    
    
}


void EKFSLAMCore::predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P, double dt, 
                            double vx, double vy, double vz, double az) {

    
    
}