#include <motor_subscriber.h>
#include <motor_publisher.h>
#include <thread>
#include <iostream>
#include "freebot_messages.h"

#include <yaml-cpp/yaml.h>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

std::ostream& operator<< (std::ostream& stream, const Position& p) {
    stream << "x: " << p.x << ", y: " << p.y << ", z: " << p.z << ", el: " << p.elevation << ", az: " << p.az;
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const Velocity& v) {
    stream << "vx: " << v.x << ", vy: " << v.y << ", vz: " << v.z << ", vel: " << v.elevation << ", vaz: " << v.az;
    return stream;
}
#include "trajectory.h"

void transform_to_position(const RigidBodyDynamics::Math::SpatialTransform &transform, Position * const position) {
    position->x = transform.r[0];
    position->y = transform.r[1];
    position->z = transform.r[2];
    auto euler = transform.E.eulerAngles(2,1,2);
    position->elevation = euler[1];
    position->az = euler[2];
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
    MotorSubscriber<ArmCommandTrajectory> sub_trajectory("arm_command_trajectory");


    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    int count = 0;
    ArmStatus status = {};
    Position &position = status.command.position;

    RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    RigidBodyDynamics::Math::VectorNd QDot = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Zero(model.q_size);
    Q[3] = -M_PI/2;
    UpdateKinematics(model, Q, QDot, QDDot);
    Position current_model_position = {};
    transform_to_position(model.X_base[model.mBodyNameMap[control_body]], &current_model_position);
    position = current_model_position;
    std::cout << position << std::endl;
    Eigen::VectorXd dx(6);
    dx.setZero();
    int last_command_num = 0;
    Trajectory trajectory;
    Position position_trajectory = position;
    Position position_sum = {};
    while(1) {
        count++;
        ArmCommand command = sub.read();        
        Velocity &velocity = command.velocity;
        ArmCommandTrajectory command_trajectory = sub_trajectory.read();
        if (command_trajectory.command_num != last_command_num) {
            last_command_num = command_trajectory.command_num;
            std::cout << "trajectory " << last_command_num << " received" << std::endl;
            trajectory.start(command_trajectory.position_trajectory, position, velocity, next_time);
            position_sum = {};
            // for (int i=0; i<command_trajectory.position_trajectory.num_points; i++) {
            //     std::cout << command_trajectory.position_trajectory.trajectory_point[i].position << std::endl;
            // }
        }
        
        if (command_trajectory.command_num) {
            position_trajectory = trajectory.get_trajectory_position(next_time);
        }
        //std::cout << position_trajectory << std::endl;

        //std::cout << model.X_base[model.mBodyNameMap[control_body]] << std::endl << std::endl;

        // requires some custom logic to deal with easy control in a 5 dof arm
        // if elevation is -90, az is in task space, else it will be just joint angle j4, az_offset is set to create no movement when transitioning 
        // a trajectory will result in az_offset of 0 
        position_sum.x += .001*velocity.x;
        position_sum.y += .001*velocity.y;
        position_sum.z += .001*velocity.z;
        position_sum.elevation += .001*velocity.elevation;
        position_sum.az += .001*velocity.az;

        position.x = position_sum.x + position_trajectory.x;
        position.y = position_sum.y + position_trajectory.y;
        position.z = position_sum.z + position_trajectory.z;
        position.elevation = position_sum.elevation + position_trajectory.elevation;
        position.az = position_sum.az + position_trajectory.az;

        UpdateKinematics(model, Q, QDot, QDDot);
        transform_to_position(model.X_base[model.mBodyNameMap[control_body]], &current_model_position);
        // dx[0] = position.ax - current_model_position.ax;
        // dx[1] = position.ay - current_model_position.ay;
        //dx[2] = position.az - current_model_position.az;
        dx[3] = position.x - current_model_position.x;
        dx[4] = position.y - current_model_position.y;
        dx[5] = position.z - current_model_position.z;
        //auto dx = current_model_position - position;
        Eigen::MatrixXd J(6,model.q_size); 
        J.setZero();
        CalcBodySpatialJacobian(model, Q, model.mBodyNameMap[control_body], J);
        Eigen::MatrixXd J2(6,model.q_size); 
        J2.setZero();
        Eigen::Vector3d point = {0,0,0};
        CalcPointJacobian6D(model, Q, model.mBodyNameMap[control_body], point, J2);

        //std::cout << J << std::endl << J2 << std::endl << std::endl;
        //Jcart=J
     //   std::cout << J << std::endl;
        // todo goal is to use Jtranspose and impedance control using joint torque, for now though position with Jinv
        Eigen::MatrixXd Jinv = J.completeOrthogonalDecomposition().pseudoInverse(); // todo look at least squares
     //   std::cout << Jinv << std::endl;
       // Eigen::VectorXd dQ = Jinv*dx;
        Eigen::VectorXd dQ = J2.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dx);
        Q += .1*dQ;
        auto Q2 = Q;
        Q2[3] += position.elevation;
        Q2[4] = position.az;

        for (int i=0; i<model.q_size; i++) {
            status.command.joint_position[i] = Q2[i];
        }

        pub.publish(status);
        next_time += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next_time);
    }

    return 0;
}