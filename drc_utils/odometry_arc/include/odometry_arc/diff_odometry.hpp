#ifndef ODOMETRY_ARC__DIFF_ODOMETRY_HPP_
#define ODOMETRY_ARC__DIFF_ODOMETRY_HPP_

#include <vector>

#include <Eigen/Dense>

#include <rclcpp/duration.hpp>

#include <odometry_arc/visibility_control.h>

namespace odometry_arc
{
    class DiffOdometry
    {
    public:
        ODOMETRY_ARC_PUBLIC
        DiffOdometry(double wheel_to_robot_center, std::vector<double> wheelRadius);

        ODOMETRY_ARC_PUBLIC
        ~DiffOdometry(void);

        ODOMETRY_ARC_PUBLIC
        void update(double leftDisp, double rightDisp, const rclcpp::Duration &duration);

        ODOMETRY_ARC_PUBLIC
        void update(const Eigen::Vector2d &deltaPhi, const Eigen::Vector2d &phiDot);

        ODOMETRY_ARC_PUBLIC
        void update(const Eigen::Vector2d &deltaPhi, const rclcpp::Duration &duration);

        ODOMETRY_ARC_PUBLIC
        double x(void) const { return x_[0]; }

        ODOMETRY_ARC_PUBLIC
        double y(void) const { return x_[1]; }

        ODOMETRY_ARC_PUBLIC
        double heading(void) const { return x_[2]; }

        ODOMETRY_ARC_PUBLIC
        void getPose(Eigen::Vector3d &x) const { x = x_; }

        ODOMETRY_ARC_PUBLIC
        void setPose(const Eigen::Vector3d &x) { x_ = x; }

        ODOMETRY_ARC_PUBLIC
        double linear(void) const { return u_[0]; }

        ODOMETRY_ARC_PUBLIC
        double angular(void) const { return u_[1]; }

        ODOMETRY_ARC_PUBLIC
        void getVelocity(Eigen::Vector2d &u) const { u = u_; }

        ODOMETRY_ARC_PUBLIC
        void setParams(double wheel_to_robot_center, std::vector<double> wheelRadius);

        ODOMETRY_ARC_PUBLIC
        void setVelocity(const Eigen::Vector2d &u) { u_ = u; }

        ODOMETRY_ARC_PUBLIC
        void resetVelocity() { u_.setZero();}

        ODOMETRY_ARC_PUBLIC
        void resetOdometry();

    private:
        std::vector<double> wheelRadius_;
        double wheel_to_robot_center_;

        Eigen::Vector2d u_;
        Eigen::Vector3d x_;
    };
}
#endif
