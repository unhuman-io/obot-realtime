#include <motor_subscriber.h>
#include <motor_publisher.h>
#include <thread>
#include <iostream>
#include "freebot_messages.h"
#include <motor_manager.h>
#include <signal.h>

#include <yaml-cpp/yaml.h>
#include "freebot_arm_control.h"
#include "freebot_base_control.h"

std::ostream& operator<< (std::ostream& stream, const Position& p) {
    stream << "x: " << p.x << ", y: " << p.y << ", z: " << p.z << ", el: " << p.elevation << ", az: " << p.az;
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const Velocity& v) {
    stream << "vx: " << v.x << ", vy: " << v.y << ", vz: " << v.z << ", vel: " << v.elevation << ", vaz: " << v.az;
    return stream;
}
#include "trajectory.h"


Eigen::VectorXd read_yaml_vector(YAML::Node node) {
    auto tmp = node.as<std::vector<double>>();
    Eigen::VectorXd out(tmp.size());
    for (int i=0; i<tmp.size(); i++) {
        out[i] = tmp[i];
    }
    return out;
}

bool run = true;

int main(int argc, char **argv) {
    YAML::Node config = YAML::LoadFile("/home/lee/freebot/freebot-realtime/config/param.yaml");

    //FreebotArmIKControl arm_control(config["arm"]["model"].as<std::string>(), config["arm"]["control_body"].as<std::string>());
    FreebotArmIKAnalyticControl arm_control(.5, .5);
    FreebotBaseControl base_control;

    auto motor_names = config["motors"].as<std::vector<std::string>>();
    Eigen::VectorXd gear_ratio = read_yaml_vector(config["gear_ratio"]);

    MotorPublisher<ArmStatus> arm_pub("arm_status");
    MotorSubscriber<ArmCommand> arm_sub("arm_command");
    MotorSubscriber<ArmCommandTrajectory> arm_sub_trajectory("arm_command_trajectory");
    MotorPublisher<BaseStatus> base_pub("base_status");
    MotorSubscriber<BaseCommand> base_sub("base_command");

    MotorManager m;
    m.get_motors_by_name(motor_names, true, true);
    std::cout << m.motors().size() << std::endl;
    // if motors are simulated set the gear ratio
    for (int i=0;i<m.motors().size();i++) {
        auto mot = m.motors()[i];
        if (typeid(*(mot.get())) == typeid(SimulatedMotor)) {
            dynamic_cast<SimulatedMotor*>(mot.get())->set_gear_ratio(gear_ratio[i]);
        }
    }


    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    int count = 0;
    ArmStatus arm_status = {};
    BaseStatus base_status = {};
    Position &position = arm_status.command.position;

    auto motor_status = m.read();
    // for (int i=0; i<motor_status.size(); i++) {
    //     Q[i] = motor_status[i].motor_position/100;
    //     if(i==0) Q[i] = -Q[i]+M_PI/2;
    //     std::cout << Q[i] << std::endl;
    // }
//    Q[3] = -M_PI/2;

    Eigen::VectorXd qm_bias = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned, Eigen::InnerStride<sizeof(motor_status[0])/sizeof(float)>>(&motor_status[0].motor_position,7).cast<double>();
    std::cout << qm_bias.transpose() << std::endl;


    int last_command_num = 0;
    Trajectory trajectory;
    Position position_trajectory = position;
    Position position_sum = {};
    signal(SIGINT,[](int signum){run = 0;});
    while(run) {
        count++;
        m.poll();
        auto motor_status = m.read();
        //std::cout << motor_status << std::endl;
        ArmCommand arm_command = arm_sub.read();
        BaseCommand base_command = base_sub.read();

        Velocity &velocity = arm_command.velocity;
        ArmCommandTrajectory arm_command_trajectory = arm_sub_trajectory.read();
        if (arm_command_trajectory.command_num != last_command_num) {
            last_command_num = arm_command_trajectory.command_num;
            std::cout << "trajectory " << last_command_num << " received" << std::endl;
            trajectory.start(arm_command_trajectory.position_trajectory, position, velocity, next_time);
            position_sum = {};
            // for (int i=0; i<command_trajectory.position_trajectory.num_points; i++) {
            //     std::cout << command_trajectory.position_trajectory.trajectory_point[i].position << std::endl;
            // }
        }
        
        if (arm_command_trajectory.command_num) {
            position_trajectory = trajectory.get_trajectory_position(next_time);
        }

        position_sum.x += .001*velocity.x;
        position_sum.y += .001*velocity.y;
        position_sum.z += .001*velocity.z;
        position_sum.elevation += .001*velocity.elevation;
        position_sum.az += .001*velocity.az;
        position_sum.y = std::max(std::min(position_sum.y, 1.), -2.);


        position.x = position_sum.x + position_trajectory.x;
        position.y = std::max(std::min(position_sum.y + position_trajectory.y, 1.), -2.);
        position.z = position_sum.z + position_trajectory.z;
        position.elevation = position_sum.elevation + position_trajectory.elevation;
        position.az = position_sum.az + position_trajectory.az;

        auto q_arm = arm_control.step(position);
        Eigen::VectorXd qd_arm = Eigen::VectorXd::Zero(5);
        q_arm[0] = position.y;
        qd_arm[0] = velocity.y;
        Eigen::VectorXd q_base = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd qd_base = base_control.step(base_command.x, base_command.az);

        Eigen::VectorXd q_max(5);
        q_max << 1, 0, 0, 0, 0;
        Eigen::VectorXd q_min(5);
        q_min << -2, 0, 0, 0, 0;

        auto in_limits = q_arm.array() >= q_max.array() || q_arm.array() <= q_min.array();
        auto q_limit = q_arm.array().max(q_min.array()).min(q_max.array());
        Eigen::Matrix<uint8_t, 5, 1> limit_command_eigen;
        limit_command_eigen = in_limits.unaryExpr([](bool b) { return static_cast<uint8_t>(b ? POSITION : POSITION); });
        qd_arm = in_limits.select(0, qd_arm);
        std::vector<uint8_t> limit_command(limit_command_eigen.data(), limit_command_eigen.data()+limit_command_eigen.size());
        // for(int i=0; i<limit_command.size(); i++) {
        //     std::cout << +limit_command[i] << ": " << q_limit[i] << ", ";
        // }
        q_arm = q_limit;

        //std::cout << q_arm[0] << " " << qd_arm[0] << std::endl;

        //std::cout << std::endl;
        std::vector<uint8_t> mode_command(2, VELOCITY);
        mode_command.insert(mode_command.end(), limit_command.begin(), limit_command.end());


        Eigen::VectorXd q(7), qd(7);
        q << q_base, q_arm;
        qd << qd_base, qd_arm;

        Eigen::VectorXd qm = q.cwiseProduct(gear_ratio) + qm_bias;
        Eigen::VectorXd qmd = qd.cwiseProduct(gear_ratio);

        for (int i=0; i<q_arm.size(); i++) {
            arm_status.command.joint_position[i] = q_arm[i];
        }

        for (int i=2; i<motor_status.size(); i++) {
            arm_status.measured.joint_position[i-2] = motor_status[i].motor_position/gear_ratio[i];
            //if(i==0) status.measured.joint_position[i] = -status.measured.joint_position[i];
        }

        base_status.measured.positionl = motor_status[0].motor_position;
        base_status.measured.positionr = motor_status[1].motor_position;
        
        // todo check this
        std::vector<float> motor_desired(qm.data(), qm.data() + qm.size());
        std::vector<float> velocity_desired(qmd.data(), qmd.data() + qmd.size());

        // for (int i=0; i<motor_desired.size(); i++) {
        //     std::cout << motor_desired[i] << std::endl;
        // }
        // std::cout << std::endl;
        // std::vector<float> motor_desired(m.motors().size());
        // for (int i=0; i<m.motors().size(); i++) {
        //     float tmp = status.command.joint_position[i];
        //     if (i==0) tmp = M_PI/2 - tmp;
        //     tmp *= 100;
        //     if (fabs(tmp - motor_status[i].motor_position) < 10) {
        //         motor_desired[i] = tmp;
        //     } else {
        //         motor_desired[i] = motor_status[i].motor_position;
        //     }
        // }

        //m.set_command_mode({VELOCITY, VELOCITY, POSITION, POSITION, POSITION, POSITION, POSITION});
        m.set_command_mode(mode_command);
        m.set_command_count(count);
        m.set_command_position(motor_desired);
        m.set_command_velocity(velocity_desired);
        m.write_saved_commands();

        arm_pub.publish(arm_status);
        base_pub.publish(base_status);
        next_time += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next_time);
    }

    m.set_command_mode(OPEN);
    m.write_saved_commands();

    return 0;
}