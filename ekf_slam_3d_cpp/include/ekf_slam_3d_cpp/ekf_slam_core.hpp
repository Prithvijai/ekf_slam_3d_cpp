#ifndef SLAM_CORE_HPP
#define SLAM_CORE_HPP


#include <Eigen/Dense>

class EKFSLAMCore {
    public:
        
    void predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P, double dt,
                             double r_v, double p_v, double y_v,  // rotation velocities 
                             double ax, double ay, double az);    // linear accelrations 
                            
    void predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P, double dt, 
                            double vx, double vy, double vz, double az);

    void associate_Landmark();
    void mesurement_update();
    void add_new_landmark();
                                     
};


#endif