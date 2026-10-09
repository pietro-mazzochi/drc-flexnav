#ifndef __DRC_POSE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP__
#define __DRC_POSE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP__
#define _USE_MATH_DEFINES

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using TimeType = std::chrono::duration<double>;

namespace DRC_PoseEstimation {

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter(const double _wheel_radius,
                       const double _wheel_halftrack, const double _theta_C_mag,
                       const double &_x0, const double &_y0,
                       const double &_theta0, const double &_v0,
                       const double &_omega0, const double &_baccx0,
                       const double &_baccy0, const double &_baccz0,
                       const double &_bgirox0, const double &_bgiroy0,
                       const double &_bgiroz0);

  bool compute(const double &_accx, const double &_accy, const double &_accz,
               const double &_girox, const double &_giroy, const double &_giroz,
               const double &_magx, const double &_magy, const double &_magz,
               const double &_right_encoder, const double &_left_encoder,
               const double &_vp, const double &_omegap, const TimeType &_T);

  Eigen::Matrix<double, 11, 11>
  get_state_Jacobian(const Eigen::Vector<double, 11> &x,
                     const Eigen::Vector2d &mu,
                     const TimeType &_T); // F

  Eigen::Matrix<double, 11, 11>
  get_measurement_Jacobian(const Eigen::Vector<double, 11> &x/*,
                            const Eigen::Vector2d &mu*/); // H

  Eigen::Vector<double, 11>
  get_state_prediction(const Eigen::Vector<double, 11> &x,
                       const Eigen::Vector2d &mu,
                       const TimeType &_T); // f

  Eigen::Vector<double, 11>
  get_measurement_prediction(const Eigen::Vector<double, 11> &x,
                             const Eigen::Vector2d &mu); // h
  //
  void set_epsilon(const double &_epsilon) { epsilon = _epsilon; }

  Eigen::Vector<double, 11> get_x_priori() { return x_priori; }
  Eigen::Vector<double, 11> get_x_state_corr() { return x_state_corr; }
  Eigen::Vector<double, 11> get_x_state_next() { return x_state_next; }

  Eigen::Matrix<double, 11, 11> get_P_priori() { return P_priori; }
  Eigen::Matrix<double, 11, 11> get_P_current() { return P_current; }
  Eigen::Matrix<double, 11, 11> get_P_next() { return P_next; }

private:
  Eigen::Matrix<double, 11, 11> F_;
  Eigen::Matrix<double, 11, 11> H_;

  Eigen::Vector<double, 11> f_;
  Eigen::Vector<double, 11> h_;

  Eigen::Vector<double, 11> x_priori, x_state_corr, x_state_next;
  Eigen::Vector<double, 11> y_;
  Eigen::Vector2d mu_;

  Eigen::Matrix<double, 11, 11> P_priori;           // previous covariance
  Eigen::Matrix<double, 11, 11> P_current;          // current covariance
  Eigen::Matrix<double, 11, 11> P_next;             // estimated next covariance
  Eigen::Matrix<double, 11, 11> P_state_estimation; // state covariance Pw
  Eigen::Matrix<double, 11, 11> P_state;            // state covariance P_est
  Eigen::Matrix<double, 11, 11> P_measure; // measuremente covariance Pv

  Eigen::Matrix<double, 11, 11> K_; // Kalman Matrix

  double wheel_radius = 0.125;
  double wheel_halftrack =
      0.54725; // distance between the wheel and the center line of the robot
  double theta_C_mag = 2.56e-01;
  double g = -9.81; // gravity

  double epsilon = 1e-9;
};

} // namespace DRC_PoseEstimation

#endif // __DRC_POSE_ESTIMATION__EXTENDED_KALMAN_FILTER_HPP__
