#include "rclcpp/rclcpp.hpp"
#include "ekf_slam_3d_cpp/ekf_slam_core.hpp"
#include <cmath> 

double EKFSLAMCore::landmark_distance(Eigen::Vector3d& l_global,Eigen::Vector3d& l_new) {

       int d;
      // this is distance b/w two points 
       d = std::sqrt( (l_global.x() - l_new.x()) * (l_global.x() - l_new.x()) + (l_global.y() - l_new.y()) * (l_global.y() - l_new.y()) + (l_global.z() - l_new.z()) * (l_global.z() - l_new.z())) ;

       return d;
}

void EKFSLAMCore::predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                            Eigen::MatrixXd& Q, double dt,
                             double r_v, double p_v, double y_v,
                             double ax, double ay, double az) {

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
    
    Eigen::Vector3d a_local(ax, ay, az);

    Eigen::Vector3d a_gobal = R * a_local;

    state(3) += r_v * dt;
    state(4) += p_v * dt;
    state(5) += y_v * dt;  //yaw

    state(6) += a_gobal.x() * dt;   // vx
    state(7) += a_gobal.y() * dt; 
    state(8) += a_gobal.z() * dt;

    state(0) += state(6) * dt + 0.5 * a_gobal.x() * dt *dt;
    state(1) += state(7) * dt + 0.5 * a_gobal.y() * dt * dt;
    state(2) += state(8) * dt + 0.5 * a_gobal.z() * dt * dt;

    state(9) = r_v;
    state(10) = p_v;
    state(11) =y_v;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(12,12);

    F(3, 9) = dt;
    F(4, 10) = dt;
    F(5, 11) = dt;
    
    F(0, 5) = - state(6) * sin(state(5)) * dt;
    F(1, 5) = state(6) * cos(state(5)) * dt;

    F.block<3, 3>(0, 6) = R * dt;

    P = F * P * F.transpose() + Q;

}


void EKFSLAMCore::predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P, 
                            Eigen::MatrixXd& Q, double dt, 
                            double vx, double vy, double vz, double az) {

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
    
    Eigen::Vector3d v_local(vx, vy, vz);

    Eigen::Vector3d v_gobal = R * v_local;


    state(0) += v_gobal.x() * dt;
    state(1) += v_gobal.y() * dt;
    state(2) += v_gobal.z() * dt;

    state(5) += az * dt;
    state(11) = az;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(12,12);

    F(3, 9) = dt;
    F(4, 10) = dt;
    F(5, 11) = dt;
    
    F(0, 5) = - state(6) * sin(state(5)) * dt;
    F(1, 5) = state(6) * cos(state(5)) * dt;

    F.block<3, 3>(0, 6) = R * dt;

    P = F * P * F.transpose() + Q;

                
    
}



void EKFSLAMCore::associate_Landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
    
    Eigen::Vector3d l(lx, ly, lz);

    Eigen::Vector3d l_gobal = (R * l) + state.head(3);

    if (state.size() <= 11) {
      this->add_new_landmark(state, P, l_gobal);
      return;
    }

    double min_d = 5000000;
    int idx = -1;

    for(int i=12; i < state.size(); i+=3)
    {
      
       Eigen::Vector3d l_new = state.segment(i, 3);
       
       double d = this->landmark_distance(l_gobal, l_new);
       
       if (d < min_d) {
          min_d = d;   // we are finding the nearest neighbour's distance and check if it is less that theshold
          idx = i;
          
       }

    }

    if (min_d <0.5) {
      this->mesurement_update(state, P, R ,idx, lx, ly, lz);
    }
    else {
       this->add_new_landmark(state, P, l_gobal);
    }
    

}

void EKFSLAMCore::mesurement_update(Eigen::VectorXd& state, Eigen::MatrixXd& P, const Eigen::Matrix3d& R, int landmark_idx,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz) {
    
    Eigen::Vector3d z(lx, ly, lz);  //local landmark position
    Eigen::Vector3d landmark_gobal = state.segment<3>(landmark_idx);
    Eigen::Vector3d robot_position = state.head(3);  // current robot position that to be updated by this landmark

    Eigen::Vector3d excepted_z = R.transpose() * (landmark_gobal - robot_position);
    
    Eigen::Vector3d y = z - excepted_z;
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

    H.block<3, 3>(0, 0) = - R.transpose();

    H.block<3, 3>(0, landmark_idx) = R.transpose();

    Eigen::Matrix3d R_noise  = Eigen::Matrix3d::Identity() * 0.01;
    Eigen:: MatrixXd S = H * P * H.transpose() + R_noise; 

    Eigen::MatrixXd K = P * H.transpose() * S.inverse();  // the  kalman gain

    // state and covariance updates 
    state = state + (K * y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
    P =  (I - K * H) * P ;


    
}

void EKFSLAMCore::add_new_landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                              Eigen::Vector3d l_gobal) { 
    
    int old_size = state.size();
    int new_size = old_size + 3;
    
    state.conservativeResize(new_size);
    state.tail(3) = l_gobal;

    P.conservativeResize(new_size, new_size);   // p = [[P, top-right], [bottom-left, bottom-right]]
    
    P.block(old_size, 0, 3, old_size).setZero();  //bottom-left of new P 
    P.block(0, old_size, old_size, 3).setZero();  // top-right of new P 

    P.block(old_size, old_size, 3, 3) = Eigen::Matrix3d::Identity() * 0.1;                     

}