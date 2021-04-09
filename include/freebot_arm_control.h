#pragma once
#include "freebot_messages.h"
#include <rbdl/rbdl.h>
#include <string>

class FreebotArmControl {
 public:
    FreebotArmControl() {}
    virtual void init(Eigen::VectorXd q) {
      q_ = q;
      qd_ = Eigen::VectorXd::Zero(q.size());
      qdd_ = Eigen::VectorXd::Zero(q.size());
    }
    virtual Eigen::VectorXd step(Position desired_position) = 0;

 protected:
    Eigen::VectorXd q_, qd_, qdd_;
    Position current_model_position_;
    unsigned int control_body_id_;
};

class FreebotArmIKControl : public FreebotArmControl {
 public:
    FreebotArmIKControl(std::string model_urdf_file, std::string control_body_name);
    virtual void init(Eigen::VectorXd q);
    virtual Eigen::VectorXd step(Position desired_position);

 protected:
    RigidBodyDynamics::Model model_;
    Position current_model_position_;
    unsigned int control_body_id_;
};

class FreebotArmIKAnalyticControl : public FreebotArmControl {
 public:
   FreebotArmIKAnalyticControl(double l1, double l2);
   virtual Eigen::VectorXd step(Position desired_position);
 protected:
   double l1_, l2_;
};