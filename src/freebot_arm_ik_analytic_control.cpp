#include "freebot_arm_control.h"
#include <stdexcept>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

FreebotArmIKAnalyticControl::FreebotArmIKAnalyticControl(double l1, double l2)
    : l1_(l1), l2_(l2) {
}

Eigen::VectorXd FreebotArmIKAnalyticControl::step(Position desired_position) {
    double &x = desired_position.x;
    double &y = desired_position.y;

    double q2 = acos((x*x + y*y - l1_*l1_ - l2_*l2_)/(2*l1_*l2_));
    q2 = std::isnan(q2) ? 0 : q2;
    q2 *= q_[2] >= 0 ? 1 : -1;
    double q1 = atan2(y,x) - atan2(l2_*sin(q2), l1_ + l2_*cos(q2));
    if (q1 < -M_PI) q1 += 2*M_PI;
    if (q1 > M_PI) q1 -= 2*M_PI;
    q_[1] = q1;
    q_[2] = q2;

    return q_;
}
