#pragma once
#include "freebot_messages.h"
#include <rbdl/rbdl.h>
#include <string>

class FreebotArmControl {
 public:
    FreebotArmControl(std::string model_urdf_file, std::string control_body_name);
    Eigen::VectorXd step(Position desired_position);

 private:
    RigidBodyDynamics::Model model_;
    RigidBodyDynamics::Math::VectorNd q_, qd_, qdd_;
    Position current_model_position_;
    unsigned int control_body_id_;
};
