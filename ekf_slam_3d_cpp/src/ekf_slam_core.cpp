#include "rclcpp/rclcpp.hpp"
#include "ekf_slam_3d_cpp/ekf_slam_core.hpp"
#include <cmath> 

double EKFSLAMCore::landmark_distance(Eigen::Vector3d& l_global,Eigen::Vector3d& l_new) {

       int d;

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
      this->add_new_landmark(state, P, lx, ly, lz);
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
      this->mesurement_update(state, P, idx, lx, ly, lz);
    }
    else {
       this->add_new_landmark(state, P, lx, ly, lz);
    }
    

}

void EKFSLAMCore::mesurement_update(Eigen::VectorXd& state, Eigen::MatrixXd& P, int landmark_idx,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz) {

    
}

void EKFSLAMCore::add_new_landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                             double lx,
                             double ly,    // local coord of potential landmark
                             double lz) {

                        

}