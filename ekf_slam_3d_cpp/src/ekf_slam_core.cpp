#include "rclcpp/rclcpp.hpp"
#include "ekf_slam_3d_cpp/ekf_slam_core.hpp"
#include <cmath> 

double EKFSLAMCore::landmark_distance(Eigen::Vector3d& l_global,Eigen::Vector3d& l_new) {

       double d;
      // this is distance b/w two points 
       d = std::sqrt( (l_global.x() - l_new.x()) * (l_global.x() - l_new.x()) + (l_global.y() - l_new.y()) * (l_global.y() - l_new.y()) + (l_global.z() - l_new.z()) * (l_global.z() - l_new.z())) ;

       return d;
}

// void EKFSLAMCore::predict_with_imu(Eigen::VectorXd& state, Eigen::MatrixXd& P,
//                             Eigen::MatrixXd& Q, double dt,
//                              double r_v, double p_v, double y_v,
//                              double ax, double ay, double az) {

//     Eigen::Matrix3d R;
//     R = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ())
//       * Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY())
//       * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX());
    
//     Eigen::Vector3d a_local(ax, ay, az);

//     Eigen::Vector3d a_gobal = R * a_local;

//     state(3) += r_v * dt;
//     state(4) += p_v * dt;
//     state(5) += y_v * dt;  //yaw

//     state(6) += a_gobal.x() * dt;   // vx
//     state(7) += a_gobal.y() * dt; 
//     state(8) += a_gobal.z() * dt;

//     state(0) += state(6) * dt + 0.5 * a_gobal.x() * dt *dt;
//     state(1) += state(7) * dt + 0.5 * a_gobal.y() * dt * dt;
//     state(2) += state(8) * dt + 0.5 * a_gobal.z() * dt * dt;

//     state(9) = r_v;
//     state(10) = p_v;
//     state(11) =y_v;

//     Eigen::MatrixXd F = Eigen::MatrixXd::Identity(P.rows(),P.cols());

//     F(3, 9) = dt;
//     F(4, 10) = dt;
//     F(5, 11) = dt;
    
//     F(0, 5) = - state(6) * sin(state(5)) * dt;
//     F(1, 5) = state(6) * cos(state(5)) * dt;

//     F.block<3, 3>(0, 6) = R * dt;

//     Eigen::MatrixXd Q_dynamic = Eigen::MatrixXd::Zero(P.rows(), P.cols());

//     Q_dynamic.block<12, 12>(0, 0) = Q;

//     P = F * P * F.transpose() + Q_dynamic;

// }


void EKFSLAMCore::predict_with_odom(Eigen::VectorXd& state, Eigen::MatrixXd& P, 
                            Eigen::MatrixXd& Q, double dt, 
                            double vx, double az) {

    double old_yaw = state(2);
    state(0) += cos(old_yaw) * vx * dt;
    state(1) += sin(old_yaw) * vx * dt;

    state(2) += az * dt;
   //  state(2) = atan2(sin(state(2)), cos(state(2)));

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(P.rows(), P.cols());  // since the state and q is increasing with landmarks  F also increase
    // F is Jaboican matrix of state vector 

    F(0, 2) = - sin(old_yaw) * vx * dt;   
    F(1, 2) =   cos(old_yaw) * vx * dt; 

    Eigen::MatrixXd Q_dynamic = Eigen::MatrixXd::Zero(P.rows(), P.cols());
    Q_dynamic.block<3, 3>(0, 0) = Q;

    P = F * P * F.transpose() + Q_dynamic;
    
}



void EKFSLAMCore::associate_Landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P,
                             Eigen::Vector3d l_local) {
    
    // 1. Calculate the rotation and global position of the candidate landmark
    Eigen::Matrix3d R_yaw;
    R_yaw << cos(state(2)), -sin(state(2)), 0,
             sin(state(2)),  cos(state(2)), 0,
             0,              0,             1;

    Eigen::Vector3d robot_position(state(0), state(1), 0.0);
    Eigen::Vector3d l_gobal = R_yaw * l_local + robot_position;

    // 2. If no landmarks exist yet, add this one immediately
    if (state.size() <= 3) {
      this->add_new_landmark(state, P, R_yaw, l_gobal);
      return;
    }

    // 3. Simple Euclidean Distance Matching
    double min_d = 1.5; // Your requested threshold
    int idx = -1;

    for(int i = 3; i < state.size(); i += 3)
    {
       Eigen::Vector3d existing_landmark = state.segment<3>(i);
       
       // Calculate Euclidean distance (std::sqrt of sum of squares)
       double d = (l_gobal - existing_landmark).norm();
       
       if (d < min_d) {
          min_d = d;
          idx = i;
       }
    }

    // 4. Update if matched, otherwise add as new
    if (idx != -1) {
      this->mesurement_update(state, P, R_yaw, idx, l_local);
    }
    else {
       this->add_new_landmark(state, P, R_yaw, l_gobal);
    }
}


void EKFSLAMCore::mesurement_update(Eigen::VectorXd& state, Eigen::MatrixXd& P, const Eigen::Matrix3d& R, int landmark_idx,
                             Eigen::Vector3d l_local) {
    
    Eigen::Vector3d landmark_gobal = state.segment<3>(landmark_idx);
    Eigen::Vector3d robot_position(state(0), state(1), 0.0);  // current robot position that to be updated by this landmark

    Eigen::Vector3d excepted_z = R.transpose() * (landmark_gobal - robot_position);
    
    Eigen::Vector3d y = l_local - excepted_z;
    
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

    H.block<3, 2>(0, 0) = - R.transpose().block<3, 2>(0, 0);  // only take x, y posiitons

    Eigen::Matrix3d dRT_dyaw;
    dRT_dyaw << -sin(state(2)),  cos(state(2)), 0,
                -cos(state(2)), -sin(state(2)), 0,
                 0,         0,        0;
    H.block<3, 1>(0, 2) = dRT_dyaw * (landmark_gobal - robot_position);
    H.block<3, 3>(0, landmark_idx) = R.transpose();
    

    Eigen::Matrix3d R_noise  = Eigen::Matrix3d::Identity() * 0.5;  //uncertanity of lidar sensor 
    Eigen:: MatrixXd S = H * P * H.transpose() + R_noise; 

    Eigen::MatrixXd K = P * H.transpose() * S.inverse();  // the  kalman gain

    // state and covariance updates 
    double d2 = y.transpose() * S.inverse() * y;

    if (d2 > 7.81) {
    return; // Stop the update. This prevents the "jumping" path.
   }
    state = state + (K * y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
    P =  (I - K * H) * P ;

}

void EKFSLAMCore::add_new_landmark(Eigen::VectorXd& state, Eigen::MatrixXd& P, const Eigen::Matrix3d& R,
                              Eigen::Vector3d l_gobal) { 
    
    int old_size = state.size();
    int new_size = old_size + 3;
    
    state.conservativeResize(new_size);
    state.tail(3) = l_gobal;

    P.conservativeResize(new_size, new_size);   // p = [[P, top-right], [bottom-left, bottom-right]]
    
    Eigen::MatrixXd Gr = Eigen::MatrixXd::Zero(3, old_size);
    
    // Identity for position (x, y)
    Gr.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity(); 
    
    // Yaw derivative for orientation (index 5)
    Eigen::Vector3d robot_position(state(0), state(1), 0.0);
    Eigen::Matrix3d dR_dyaw;
    dR_dyaw << -sin(state(2)), -cos(state(2)), 0,
                cos(state(2)), -sin(state(2)), 0,
                0,         0,        0;
   //  Gr.block<3, 1>(0, 2) = dR_dyaw * (l_gobal - robot_position);
    Eigen::Vector3d l_local = R.transpose() * (l_gobal - robot_position);
    Gr.block<3, 1>(0, 2) = dR_dyaw * l_local;

    P.block(old_size, 0, 3, old_size) = Gr * P.topLeftCorner(old_size, old_size);
    P.block(0, old_size, old_size, 3) = P.block(old_size, 0, 3, old_size).transpose();

    Eigen::Matrix3d R_sensor = Eigen::Matrix3d::Identity() * 0.1; 
    P.block(old_size, old_size, 3, 3) = Gr * P.topLeftCorner(old_size, old_size) * Gr.transpose() + R_sensor;
}