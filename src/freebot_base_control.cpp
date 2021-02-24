#include "freebot_base_control.h"



Eigen::Vector2d FreebotBaseControl::step(double x, double az) {
    Eigen::Vector2d out;
    out << x-az, x+az;
    return out;
}
