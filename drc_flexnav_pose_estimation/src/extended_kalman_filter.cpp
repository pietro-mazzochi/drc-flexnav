
#include "drc_flexnav_pose_estimation/extended_kalman_filter.hpp"
#include <Eigen/LU>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace DRC_PoseEstimation {
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const double _wheel_radius, const double _wheel_halftrack,
    const double _theta_C_mag, const double &_x0, const double &_y0,
    const double &_theta0, const double &_v0, const double &_omega0,
    const double &_baccx0, const double &_baccy0, const double &_baccz0,
    const double &_bgirox0, const double &_bgiroy0, const double &_bgiroz0)
    : wheel_radius{_wheel_radius}, wheel_halftrack{_wheel_halftrack},
      theta_C_mag{_theta_C_mag} {

  x_priori << _x0, _y0, _theta0, _v0, _omega0, _baccx0, _baccy0, _baccz0,
      _bgirox0, _bgiroy0, _bgiroz0;

  P_priori << 1e+1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
      0, 1e+1, 0, 0, 0, 0, 0, 0, 0, 0, 0,         //
      0, 0, 1e+1, 0, 0, 0, 0, 0, 0, 0, 0,         //
      0, 0, 0, 1e+1, 0, 0, 0, 0, 0, 0, 0,         //
      0, 0, 0, 0, 1e+1, 0, 0, 0, 0, 0, 0,         //
      0, 0, 0, 0, 0, 1e+1, 0, 0, 0, 0, 0,         //
      0, 0, 0, 0, 0, 0, 1e+1, 0, 0, 0, 0,         //
      0, 0, 0, 0, 0, 0, 0, 1e+1, 0, 0, 0,         //
      0, 0, 0, 0, 0, 0, 0, 0, 1e+1, 0, 0,         //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1e+1, 0,         //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e+1;         //

  P_state_estimation << 1e-3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
      0, 1e-3, 0, 0, 0, 0, 0, 0, 0, 0, 0,                   //
      0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 0, 0,                   //
      0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 0,                   //
      0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0,                   //
      0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0,                   //
      0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0,                   //
      0, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0,                   //
      0, 0, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0,                   //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-3, 0,                   //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-3;                   //

  P_measure << 528.3e-6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //
      0, 528.3e-6, 0, 0, 0, 0, 0, 0, 0, 0, 0,          //
      0, 0, 528.3e-6, 0, 0, 0, 0, 0, 0, 0, 0,          //
      0, 0, 0, 3.3e-3, 0, 0, 0, 0, 0, 0, 0,            //
      0, 0, 0, 0, 3.3e-3, 0, 0, 0, 0, 0, 0,            //
      0, 0, 0, 0, 0, 3.3e-3, 0, 0, 0, 0, 0,            //
      0, 0, 0, 0, 0, 0, 52.82e+9, 0, 0, 0, 0,          //
      0, 0, 0, 0, 0, 0, 0, 52.82e+9, 0, 0, 0,          //
      0, 0, 0, 0, 0, 0, 0, 0, 52.82e+9, 0, 0,          //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1.73e-5, 0,           //
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.73e-5;           //
}
bool ExtendedKalmanFilter::compute(
    const double &_accx, const double &_accy, const double &_accz,
    const double &_girox, const double &_giroy, const double &_giroz,
    const double &_magx, const double &_magy, const double &_magz,
    const double &_right_encoder, const double &_left_encoder,
    const double &_vp, const double &_omegap, const TimeType &_T) {
  y_ << _accx, _accy, _accz, _girox, _giroy, _giroz, _magx, _magy, _magz,
      _left_encoder, _right_encoder;

  mu_ << _vp, _omegap;

  // h(x ̂(k│k-1),u(k))
  h_ = get_measurement_prediction(x_priori, mu_);

  // H(x(k),u(k))
  H_ = get_measurement_Jacobian(x_priori /*, mu_*/);
  auto H_T = H_.transpose();

  // K[k]
  auto H = H_ * P_priori * H_T + P_measure;

  if (std::fabs(H.determinant()) < epsilon) {
    std::ostringstream oss;
    oss << std::scientific;
    oss.precision(4);
    oss << "H determinant is too close to zero, invalid inversion. |H| = "
        << H.determinant() << ", epsilon: " << epsilon << ". \n H:\n " << H
        << ". \n H_:\n " << H_ << ". \n P_measure:\n " << P_measure
        << ". \n P_priori:\n " << P_priori;
    throw std::domain_error(oss.str());
  }

  K_ = P_priori * H_T * H.inverse();

  // x^(k|k)
  x_state_corr = x_priori + K_ * (y_ - h_);

  // P(k|k)
  P_current = (Eigen::Matrix<double, 11, 11>::Identity() - K_ * H_) * P_priori;

  // x(k+1|k)
  x_state_next = get_state_prediction(x_state_corr, mu_, _T);

  // P(k+1|k)
  P_next = get_state_Jacobian(x_state_next, mu_, _T) * P_current *
               (get_state_Jacobian(x_state_corr, mu_, _T).transpose()) +
           P_state_estimation;

  // Update step
  x_priori = x_state_next;
  P_priori = P_next;

  return true;
}

Eigen::Matrix<double, 11, 11>
ExtendedKalmanFilter::get_state_Jacobian(const Eigen::Vector<double, 11> &x,
                                         const Eigen::Vector2d &mu,
                                         const TimeType &_T) { // F
  double T = _T.count(); // duration in seconds
  //-T*(v+(T*mu_1)/2)*sin(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_13 = -T * (x(3) + (T * mu(0)) / 2) *
              sin(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);
  // T*cos(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_14 = T * cos(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);
  //-((T^2)/8)*(2*v+(T*mu_1))*sin(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_15 = -((T * T) / 4) * (2 * x(3) + (T * mu(0))) *
              sin(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);

  // T*(v+(T*mu_1)/2)*cos(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_23 = T * (x(3) + (T * mu(0)) / 2) *
              cos(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);
  // T*sin(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_24 = T * sin(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);
  //((T^2)/8)*(2*v+(T*mu_1))*cos(theta+(T*omega)/2+((T^2)*mu_2)/4)
  auto F_25 = ((T * T) / 4) * (2 * x(3) + (T * mu(0))) *
              cos(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);

  Eigen::Matrix<double, 11, 11> Fj_matrix;

  Fj_matrix << 1, 0, F_13, F_14, F_15, 0, 0, 0, 0, 0, 0, // px
      0, 1, F_23, F_24, F_25, 0, 0, 0, 0, 0, 0,          // py
      0, 0, 1, 0, T, 0, 0, 0, 0, 0, 0,                   // theta
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,                   // v
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,                   // omega
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,                   // B acc x
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,                   // B acc y
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,                   // B acc z
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,                   // B giro x
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,                   // B giro y
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;                   // B giro z

  return Fj_matrix;
}

Eigen::Matrix<double, 11, 11>
ExtendedKalmanFilter::get_measurement_Jacobian(const Eigen::Vector<double, 11> &x/*,
                                       const Eigen::Vector2d &mu*/) { // H

  Eigen::Matrix<double, 11, 11> Hj_matrix =
      Eigen::Matrix<double, 11, 11>::Zero();
  Hj_matrix(0, 5) = 1;
  Hj_matrix(1, 6) = 1;
  Hj_matrix(2, 7) = 1;
  Hj_matrix(3, 8) = 1;
  Hj_matrix(4, 9) = 1;
  Hj_matrix(5, 10) = 1;

  Hj_matrix(5, 4) = 1;

  Hj_matrix(6, 2) =
      -sin(x(2)) * sin(theta_C_mag + M_PI / 2) +
      cos(x(2)) * cos(theta_C_mag + M_PI / 2); // magx = -sin(theta);
  Hj_matrix(7, 2) =
      +cos(x(2)) * sin(theta_C_mag + M_PI / 2) -
      sin(x(2)) * cos(theta_C_mag + M_PI / 2); // magy = cos(theta);

  Hj_matrix(9, 3) = 1 / wheel_radius;               // d encoder r/dv
  Hj_matrix(9, 4) = wheel_halftrack / wheel_radius; // d encoder r/dw

  Hj_matrix(10, 3) = 1 / wheel_radius;                // l encoder l/dv
  Hj_matrix(10, 4) = -wheel_halftrack / wheel_radius; // l encoder l/dw

  return Hj_matrix;
}

Eigen::Vector<double, 11>
ExtendedKalmanFilter::get_state_prediction(const Eigen::Vector<double, 11> &x,
                                           const Eigen::Vector2d &mu,
                                           const TimeType &_T) { // f
  double T = _T.count(); // duration in seconds

  auto f_m_1 = x(0) + T * (x(3) + (T * mu(0)) / 2) *
                          cos(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1) / 4));
  auto f_m_2 = x(1) + T * (x(3) + (T * mu(0)) / 2) *
                          sin(x(2) + (T * x(4)) / 2 + ((T * T) * mu(1)) / 4);
  auto f_m_3 = x(2) + T * x(4) + ((T * T) * mu(1)) / 2;
  auto f_m_4 = x(3) + T * mu(0);
  auto f_m_5 = x(4) + T * mu(1);

  Eigen::Vector<double, 11> f_m;
  f_m << f_m_1, // px = px + T*(v+(T*mu_1)/2)*cos(theta + (T*omega)/2 +
                // ((T^2)*mu_2)/4);
      f_m_2,    // py = py + T*(v+(T*mu_1)/2)*sin(theta + (T*omega)/2 +
                // ((T^2)*mu_2)/4);
      f_m_3,    // theta = theta + T*omega + ((T^2)*mu_2)/2;
      f_m_4,    // v = v + T*mu_1;
      f_m_5,    // omega = omega + T*mu_2;
      x(5),     // baccx
      x(6),     // baccy
      x(7),     // baccz
      x(8),     // bgirox
      x(9),     // bgiroy
      x(10);    // bgiroz

  return f_m;
}

Eigen::Vector<double, 11> ExtendedKalmanFilter::get_measurement_prediction(
    const Eigen::Vector<double, 11> &x,
    const Eigen::Vector2d &mu) { // h

  Eigen::Vector<double, 11> h_m;

  h_m << mu(0) + x(5), // accx = mu_1 + b_accx
      x(6),            // accy = b_accy
      g + x(7),        // accz = g + b_accz
      x(8),            // girox = b_girox
      x(9),            // giroy = b_giroy
      x(4) + x(10),    // giroz = omega + b_giroz
      cos(x(2)) * sin(theta_C_mag + M_PI / 2) +
          sin(x(2)) * cos(theta_C_mag + M_PI / 2), // magx = sin(theta)
      sin(x(2)) * sin(theta_C_mag + M_PI / 2) +
          cos(x(2)) * cos(theta_C_mag + M_PI / 2), // magy = cos(theta)
      0,                                           // magz = 0
      (x(3) + wheel_halftrack * x(4)) /
          wheel_radius, // encoder_d = (v/r) + (b*omega/r)
      (x(3) - wheel_halftrack * x(4)) /
          wheel_radius; // encoder_e = (v/r) - (b*omega/r)

  return h_m;
}

} // namespace DRC_PoseEstimation
