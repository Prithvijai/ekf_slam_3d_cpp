#ifndef SLAM_CORE_HPP
#define SLAM_CORE_HPP


#include <Eigen/Dense>
#include <limits>

class EKFSLAMCore {
    public:
   

    // void predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P,
    //                         Eigen::MatrixXd& Q, double dt,
    //                          double r_v, double p_v, double y_v,  // rotation velocities 
    //                          double ax, double ay, double az);    // linear accelrations 
                            
    void predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                            Eigen::MatrixXd& Q, double dt, 
                            double vx, double az); // linear velocitiy  & angular accelrations 

    void associate_Landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                            Eigen::Vector3d l_local);// local coord of potential landmark
    
    void mesurement_update(Eigen::VectorXd& state, Eigen::MatrixXd& P, const Eigen::Matrix3d& R,int landmark_idx,
                             Eigen::Vector3d l_local);
                             
    void add_new_landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,const Eigen::Matrix3d& R,
                             Eigen::Vector3d l_gobal);
                                     
    
    double landmark_distance(Eigen::Vector3d& l_global, Eigen::Vector3d& l_new);
};


#endif