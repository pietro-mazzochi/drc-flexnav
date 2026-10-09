#include <odometry_arc/diff_odometry.hpp>
#include <iostream>

namespace odometry_arc
{
        DiffOdometry::DiffOdometry(double wheel_to_robot_center, std::vector<double> wheelRadius)
        {
                wheel_to_robot_center_ = wheel_to_robot_center;
                wheelRadius_ = wheelRadius;
                u_.setZero();
                x_.setZero();
        }

        DiffOdometry::~DiffOdometry(void)
        {

        }

        // Incremental encoders sense angular displacement and
        // not velocity
        // deltaPhi[0] is the left wheel angular displacement
        // deltaPhi[1] is the right wheel angular displacement
        void DiffOdometry::update(const Eigen::Vector2d & deltaPhi, const Eigen::Vector2d & phiDot)
        {
                Eigen::Vector2d deltaU;
                deltaU << (deltaPhi[0]*wheelRadius_[0]+deltaPhi[1]*wheelRadius_[1])/2.0,
                          (deltaPhi[1]*wheelRadius_[1]-deltaPhi[0]*wheelRadius_[0])/(wheel_to_robot_center_ * 2);

                double deltaS=(fabs(deltaU[1]) > std::numeric_limits<double>::epsilon())?
                       deltaU[0]*sin(deltaU[1]/2)/(deltaU[1]/2):deltaU[0];

                u_ << (phiDot[0]*wheelRadius_[0]+phiDot[1]*wheelRadius_[1])/2.0,
                      (phiDot[1]*wheelRadius_[1]-phiDot[0]*wheelRadius_[0])/(wheel_to_robot_center_ * 2);

                Eigen::Vector3d deltaX;
                deltaX << deltaS*cos(x_[2]+deltaU[1]/2.0),
                        deltaS*sin(x_[2]+deltaU[1]/2.0),
                        deltaU[1];

                x_ += deltaX;
        }
        void DiffOdometry::update(const Eigen::Vector2d & deltaPhi,const rclcpp::Duration & duration)
        {
                double dt = duration.nanoseconds()*1e-9;

                Eigen::Vector2d deltaU;
                deltaU << (deltaPhi[0]*wheelRadius_[0] + deltaPhi[1]*wheelRadius_[1])/2.0,
                          (deltaPhi[1]*wheelRadius_[1] - deltaPhi[0]*wheelRadius_[0])/(wheel_to_robot_center_ * 2);

                double deltaS = (fabs(deltaU[1]) > std::numeric_limits<double>::epsilon()) ?
                                deltaU[0]*sin(deltaU[1]/2)/(deltaU[1]/2) : deltaU[0];

                if (dt > std::numeric_limits<double>::epsilon())
                        u_ << deltaS/dt , deltaU[1]/dt;

                Eigen::Vector3d deltaX;
                deltaX << deltaS*cos(x_[2]+deltaU[1]/2.0),
                          deltaS*sin(x_[2]+deltaU[1]/2.0),
                          deltaU[1];

                x_ += deltaX;
        }

        void DiffOdometry::update(double leftDisp, double rightDisp, const rclcpp::Duration & duration)
        {
                Eigen::Vector2d deltaPhi;
                deltaPhi << leftDisp, rightDisp;

                update(deltaPhi, duration);
        }

        void DiffOdometry::setParams(double wheel_to_robot_center, std::vector<double> wheelRadius)
        {
                wheel_to_robot_center_ = wheel_to_robot_center;
                wheelRadius_ = wheelRadius;
        }

        void DiffOdometry::resetOdometry()
        {
                u_.setZero();
                x_.setZero();
        }
}
