#include<iostream>
#include "rclcpp/rclcpp.hpp"   // main lib for writing the ros2 cpp 
#include "ekf_slam_3d_cpp/ekf_slam_node.hpp"

int main(int argc, char * argv[])    // argc (count) and argv[] (arguement vector) is standard way to recive command-line arguements 
{
    std::cout<<"Hello world" << std::flush; 
    rclcpp::init(argc, argv);   // this allows for change in topic names and other things from terminal and initialize the ROS2 node
    auto node = std::make_shared<EKFSLAMNode>();  // this make_shard is shared ptr for the class EKFSLAMNode 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}