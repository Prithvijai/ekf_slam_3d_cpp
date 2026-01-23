#include "ekf_slam_3d_cpp/ekf_slam_node.hpp"

EKFSLAMNode::EKFSLAMNode() : Node("ekf_slam_node") {

    RCLCPP_INFO(this->get_logger(), "Hello , this is SLAM node");  // log print for ros2 , should used after ros2 init 

    // subcription to the topic odom and odom_sub_ is the SharedPtr which holds this subcription
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "diff_cont/odom",
                            10,
                        std::bind(&EKFSLAMNode::odom_callback, this, std::placeholders::_1)  // this bridge the callback to this subcription 
                    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                        "imu",
                            10,
                        std::bind(&EKFSLAMNode::imu_callback, this, std::placeholders::_1) 
                    );

    point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "points",
                            10,
                        std::bind(&EKFSLAMNode::point_callback, this, std::placeholders::_1) 
                    );

    state_ = Eigen::VectorXd::Zero(12);   // [x, y, z, roll, pitch, yaw, 6 other velcoities ] for the state vector
    
    p_ = Eigen::MatrixXd::Identity(12, 12) * 0.01;

    Q_ = Eigen::MatrixXd::Identity(12, 12) * 0.001;
    
}

void EKFSLAMNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    //odom contain the position of robot from the origin or start , and what velocity is applied to it. 
    // RCLCPP_INFO(this->get_logger(), "odom pos: [%.2f, %.2f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "odom speed: [%.2f, %.2f]", msg->twist.twist.linear.x , msg->twist.twist.angular.z);
    double dt = (this->now() - last_time_).seconds();
    last_time_ = this->now();

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    double wz = msg->twist.twist.angular.z;

    slam_core_->predict_with_odom(state_, p_,Q_ ,dt, vx, vy, vz, wz);

}
void EKFSLAMNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr  msg) {
    // RCLCPP_INFO(this->get_logger(), "imu x [ %.2f]", msg->linear_acceleration.x);
    // RCLCPP_INFO(this->get_logger(), "imu angluar z [ %.2f]", msg->angular_velocity.z);
    double dt = (this->now() - last_time_).seconds();
    last_time_ =this->now();

    double roll_v = msg->angular_velocity.x;
    double pitch_v = msg->angular_velocity.y;
    double yaw_v = msg->angular_velocity.z;

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    slam_core_->predict_with_imu(state_, p_, Q_, dt, roll_v, pitch_v, yaw_v, ax, ay, az);

}
void EKFSLAMNode::point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "point");

}