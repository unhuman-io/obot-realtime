#pragma once
#include "freebot_messages.h"
#include "Eigen/Dense"
#include <string>

class FreebotBaseControl {
 public:
    FreebotBaseControl() {}
    // qd
    Eigen::Vector2d step(double x, double az);

 private:

};
