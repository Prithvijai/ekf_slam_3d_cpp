#ifndef SLAM_CORE_HPP
#define SLAM_CORE_HPP


#include <Eigen/Dense>

class EKFSLAMCore {
    public:
   

    void predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                            Eigen::MatrixXd& Q, double dt,
                             double r_v, double p_v, double y_v,  // rotation velocities 
                             double ax, double ay, double az);    // linear accelrations 
                            
    void predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                            Eigen::MatrixXd& Q, double dt, 
                            double vx, double vy, double vz, double az); // linear velocitiy  & angular accelrations 

    void associate_Landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz);
    
    void mesurement_update(Eigen::VectorXd& state, Eigen::MatrixXd& P, int landmark_idx,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz);
                             
    void add_new_landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz);
                                     
    
    double landmark_distance(Eigen::Vector3d& l_global, Eigen::Vector3d& l_new);
};


#endif