#include "obot_arm_control.h"
#include <stdexcept>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

void transform_to_position(const RigidBodyDynamics::Math::SpatialTransform &transform, Position * const position) {
    position->x = transform.r[0];
    position->y = transform.r[1];
    position->z = transform.r[2];
    auto euler = transform.E.eulerAngles(2,1,2);
    position->elevation = euler[1];
    position->az = euler[2];
}

ObotArmIKControl::ObotArmIKControl(std::string model_urdf_file, std::string control_body_name) {
    rbdl_check_api_version(RBDL_API_VERSION);

    if (!RigidBodyDynamics::Addons::URDFReadFromFile (model_urdf_file.c_str(), &model_, false)) {
        throw std::runtime_error("Error loading urdf model: " + model_urdf_file);
    }
    control_body_id_ = model_.mBodyNameMap[control_body_name];

    std::cout << "Arm control using RBDL version: " << RBDL_API_VERSION << std::endl;
    std::cout << "Arm URDF: " << model_urdf_file << std::endl;
    std::cout << "Arm control body: " << control_body_name << std::endl;

    auto q = RigidBodyDynamics::Math::VectorNd::Zero(model_.q_size);
    init(q);

    //std::cout << current_model_position_ << std::endl;
    //std::cout << q_ << std::endl;
}

void ObotArmIKControl::init(Eigen::VectorXd q) {
    ObotArmControl::init(q);
    UpdateKinematics(model_, q_, qd_, qdd_);
    transform_to_position(model_.X_base[control_body_id_], &current_model_position_);
}


// requires some custom logic to deal with easy control in a 5 dof arm
// if elevation is -90, az is in task space, else it will be just joint angle j4, az_offset is set to create no movement when transitioning 
// a trajectory will result in az_offset of 0 
Eigen::VectorXd ObotArmIKControl::step(Position desired_position) {
    UpdateKinematics(model_, q_, qd_, qdd_);
    transform_to_position(model_.X_base[control_body_id_], &current_model_position_);
    Eigen::VectorXd dx(6);
    dx.setZero();
    // dx[0] = position.ax - current_model_position.ax;
    // dx[1] = position.ay - current_model_position.ay;
    //dx[2] = position.az - current_model_position.az;
    dx[3] = desired_position.x - current_model_position_.x;
    dx[4] = desired_position.y - current_model_position_.y;
    dx[5] = desired_position.z - current_model_position_.z;
    //auto dx = current_model_position - position;
    Eigen::MatrixXd J(6,model_.q_size); 
    J.setZero();
    CalcBodySpatialJacobian(model_, q_, control_body_id_, J);
    Eigen::MatrixXd J2(6,model_.q_size); 
    J2.setZero();
    Eigen::Vector3d point = {0,0,0};
    CalcPointJacobian6D(model_, q_, control_body_id_, point, J2);

    //std::cout << J << std::endl << J2 << std::endl << std::endl;
    //Jcart=J
    //   std::cout << J << std::endl;
    // todo goal is to use Jtranspose and impedance control using joint torque, for now though position with Jinv
    Eigen::MatrixXd Jinv = J.completeOrthogonalDecomposition().pseudoInverse(); // todo look at least squares
    //   std::cout << Jinv << std::endl;
    // Eigen::VectorXd dQ = Jinv*dx;
    Eigen::VectorXd dQ = J2.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dx);
    q_ += .1*dQ;
    auto Q2 = q_;
    Q2[3] += desired_position.elevation;
    Q2[4] = desired_position.az;
    return Q2;
}
