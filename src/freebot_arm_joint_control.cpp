#include "freebot_arm_control.h"
//#include <stdexcept>

FreebotArmControl::FreebotArmControl(std::string model_urdf_file, std::string control_body_name) {

}

void FreebotArmControl::init(Eigen::VectorXd q) {

}

Eigen::VectorXd FreebotArmControl::step(ArmJointVelocity velocity) {
    qd_ = velocity;
    if (q_ > max pos)
        mode = pos;
        q = max pos
    if (q_ < min pos)
        ...
    
    return Q2;
}
