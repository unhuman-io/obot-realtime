#include "obot_arm_control.h"
//#include <stdexcept>

ObotArmControl::ObotArmControl(std::string model_urdf_file, std::string control_body_name) {

}

void ObotArmControl::init(Eigen::VectorXd q) {

}

Eigen::VectorXd ObotArmControl::step(ArmJointVelocity velocity) {
    qd_ = velocity;
    if (q_ > max pos)
        mode = pos;
        q = max pos
    if (q_ < min pos)
        ...
    
    return Q2;
}
