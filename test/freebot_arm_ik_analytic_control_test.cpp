#include <gtest/gtest.h>
#include "freebot_arm_control.h"
#include <iostream>

TEST(IKAnalyticTest, Basic) {
    double l1 = .5;
    double l2 = .6;
    FreebotArmIKAnalyticControl c(l1, l2);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(5);
    c.init(q);
    for (double q1=-M_PI*.9; q1<M_PI*.9; q1+=.1) {
        for (double q2=0; q2<M_PI*.9; q2+=.1) {
            double x = l1*cos(q1) + l2*cos(q2+q1);
            double y = l1*sin(q1) + l2*sin(q2+q1);
            Position p{.x = x, .y = y};
            q = c.step(p);
            std::cout << x << " " << y << " " << q1 << " " << q2 << q.transpose() << std::endl;
            ASSERT_NEAR(q[1], q1, std::numeric_limits<double>::epsilon()*1e8);
            ASSERT_NEAR(q[2], q2, std::numeric_limits<double>::epsilon()*1e9);
        }
    }
    Position p{.x = 1, .y = 1};
    q = c.step(p);
    ASSERT_NEAR(q[1], M_PI_4, std::numeric_limits<double>::epsilon()*100);
    ASSERT_DOUBLE_EQ(q[2], 0);
}