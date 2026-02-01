#include "ekf_slam_3d_cpp/ekf_slam_node.hpp"

EKFSLAMNode::EKFSLAMNode() : Node("ekf_slam_node") {

    RCLCPP_INFO(this->get_logger(), "Hello , this is SLAM node");  // log print for ros2 , should used after ros2 init 

    // subcription to the topic odom and odom_sub_ is the SharedPtr which holds this subcription
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "diff_cont/odom",
                            10,
                        std::bind(&EKFSLAMNode::odom_callback, this, std::placeholders::_1)  // this bridge the callback to this subcription 
                    );

    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //                     "imu",
    //                         10,
    //                     std::bind(&EKFSLAMNode::imu_callback, this, std::placeholders::_1) 
    //                 );

    point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        "points",
                            10,
                        std::bind(&EKFSLAMNode::point_callback, this, std::placeholders::_1) 
                    );

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
                        "path",
                            10 
                    );

    landmark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "landmarks",
                            10 
                    );

    path_history_.header.frame_id = "odom";
    state_ = Eigen::VectorXd::Zero(3);   // [x, y, yaw ] for the state vector (corrected) previous 12 state to remove drifts 

    last_time_ = this->now();
    
    p_ = Eigen::MatrixXd::Identity(3, 3) * 0.01;

    Q_ = Eigen::MatrixXd::Identity(3, 3) * 0.001;
    
}

void EKFSLAMNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //odom contain the position of robot from the origin or start , and what velocity is applied to it. 
    // RCLCPP_INFO(this->get_logger(), "odom pos: [%.2f, %.2f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "odom speed: [%.2f, %.2f]", msg->twist.twist.linear.x , msg->twist.twist.angular.z);

    rclcpp::Time current_time = msg->header.stamp;


    if (last_time_.get_clock_type() != current_time.get_clock_type() || last_time_.nanoseconds() == 0) {
        last_time_ = current_time;
        return;
    } 

    double dt = (rclcpp::Time(msg->header.stamp) - last_time_).seconds();
    last_time_ = rclcpp::Time(msg->header.stamp);

    double vx = msg->twist.twist.linear.x;
    double wz = msg->twist.twist.angular.z;
    RCLCPP_INFO(this->get_logger(), "Teleop control: %.2f, %.2f",vx, wz);

    slam_core_->predict_with_odom(state_, p_,Q_ ,dt, vx, wz);
    // state_(2) = 0;
    // RCLCPP_INFO(this->get_logger(), "New landmark added! Map size: %ld", state_.size());

}
// void EKFSLAMNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr  msg) {
//     // RCLCPP_INFO(this->get_logger(), "imu x [ %.2f]", msg->linear_acceleration.x);
//     // RCLCPP_INFO(this->get_logger(), "imu angluar z [ %.2f]", msg->angular_velocity.z);

//     rclcpp::Time current_time = msg->header.stamp;


//     if (last_time_.get_clock_type() != current_time.get_clock_type() || last_time_.nanoseconds() == 0) {
//         last_time_ = current_time;
//         return;
//     }

//     double dt = (rclcpp::Time(msg->header.stamp) - last_time_).seconds();
//     last_time_ =rclcpp::Time(msg->header.stamp);

//     double roll_v = msg->angular_velocity.x;
//     double pitch_v = msg->angular_velocity.y;
//     double yaw_v = msg->angular_velocity.z;

//     double ax = msg->linear_acceleration.x;
//     double ay = msg->linear_acceleration.y;
//     double az = msg->linear_acceleration.z;

//     slam_core_->predict_with_imu(state_, p_, Q_, dt, roll_v, pitch_v, yaw_v, ax, ay, az);

//     // RCLCPP_INFO(this->get_logger(), "New landmark added! Map size: %ld", state_.size());

// }
void EKFSLAMNode::point_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud);

    std::vector<Eigen::Vector3d> landmarks; //multiple landmarks 
    // Eigen::Vector3d landmark = Eigen::Vector3d::Zero();
    // feature_->feature_extraction_gazebo(*cloud, landmarks);
    feature_->feature_extraction_multiple_gazebo(*cloud, landmarks);

    for (const auto& lm : landmarks) {
        double d = lm.norm();
        if (d < 10.0) {
            slam_core_->associate_Landmark(state_, p_,lm);
        }
    }
    // slam_core_->associate_Landmark(state_, p_, landmarks.x(), landmarks.y(), landmarks.z());
    publish_path();
    publish_landmark();
    
    RCLCPP_INFO(this->get_logger(), "New landmark added! Map size: %ld, Pos: %.2f, %.2f, Z: %.2f", state_.size(), state_.x(), state_.y(), state_.z());
}



void EKFSLAMNode::publish_path() {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "odom";
    pose.pose.position.x = state_(0);
    pose.pose.position.y = state_(1);
    pose.pose.position.z = 0.0;

    path_history_.poses.push_back(pose);
    path_pub_->publish(path_history_);

}
void EKFSLAMNode::publish_landmark() {
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i =3; i < state_.size(); i+= 3) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.id = i;

        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = state_(i);
        marker.pose.position.y = state_(i+1);
        marker.pose.position.z = state_(i+2);

        marker.scale.x = 0.1; marker.scale.y = 0.1; marker.scale.z = 0.1;
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    landmark_pub_->publish(marker_array);
}