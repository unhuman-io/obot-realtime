#include <motor_subscriber.h>
#include <motor_publisher.h>
#include <thread>
#include <iostream>
#include "freebot_messages.h"
#include <yaml-cpp/yaml.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

void transform_to_position(const RigidBodyDynamics::Math::SpatialTransform &transform, Position * const position) {
    position->x = transform.r[0];
    position->y = transform.r[1];
    position->z = transform.r[2];
    //position->ax = transform.E[2,1];
}


int main(int argc, char **argv) {
    YAML::Node config = YAML::LoadFile("/home/lee/unhuman/freebot-realtime/config/param.yaml");
    rbdl_check_api_version (RBDL_API_VERSION);
    RigidBodyDynamics::Model model;
    auto model_urdf_file = config["model"].as<std::string>();
    auto control_body = config["control_body"].as<std::string>();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile (model_urdf_file.c_str(), &model, false)) {
        std::cerr << "Error loading model " << model_urdf_file << std::endl;
        abort();
    }

    MotorPublisher<ArmStatus> pub("arm_status");
    MotorSubscriber<ArmCommand> sub("arm_command");


    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    int count = 0;
    ArmStatus status = {};
    Position &position = status.command.position;

    RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    RigidBodyDynamics::Math::VectorNd QDot = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    UpdateKinematics(model, Q, QDot, QDDot);
    Position current_model_position = {};
    transform_to_position(model.X_base[model.mBodyNameMap[control_body]], &current_model_position);
    position = current_model_position;
    Eigen::VectorXd dx(6);
    dx.setZero();
    while(1) {
        count++;
        ArmCommand command = sub.read();
        Velocity &velocity = command.velocity;

        std::cout << position.x << std::endl;

        position.x += .001*velocity.x;
        position.y += .001*velocity.y;
        position.z += .001*velocity.z;
        position.ax += .001*velocity.ax;
        position.az += .001*velocity.az;

        UpdateKinematics(model, Q, QDot, QDDot);
        transform_to_position(model.X_base[model.mBodyNameMap[control_body]], &current_model_position);
        dx[3] = position.x - current_model_position.x;
        dx[4] = position.y - current_model_position.y;
        dx[5] = position.z - current_model_position.z;
        //auto dx = current_model_position - position;
        Eigen::MatrixXd J(6,model.q_size); 
        J.setZero();
        CalcBodySpatialJacobian(model, Q, model.mBodyNameMap[control_body], J);
        //Jcart=J
     //   std::cout << J << std::endl;
        // todo goal is to use Jtranspose and impedance control using joint torque, for now though position with Jinv
        Eigen::MatrixXd Jinv = J.completeOrthogonalDecomposition().pseudoInverse(); // todo look at least squares
     //   std::cout << Jinv << std::endl;
        Q += .1*Jinv*dx;

        for (int i=0; i<model.q_size; i++) {
            status.command.joint_position[i] = Q[i];
        }

        pub.publish(status);
        next_time += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next_time);
    }

    return 0;
}