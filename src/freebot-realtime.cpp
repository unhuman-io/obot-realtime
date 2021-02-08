#include <motor_subscriber.h>
#include <motor_publisher.h>
#include <thread>
#include <iostream>
#include "freebot_messages.h"
#include <motor_manager.h>

#include <yaml-cpp/yaml.h>
#include "freebot_arm_control.h"

std::ostream& operator<< (std::ostream& stream, const Position& p) {
    stream << "x: " << p.x << ", y: " << p.y << ", z: " << p.z << ", el: " << p.elevation << ", az: " << p.az;
    return stream;
}

std::ostream& operator<< (std::ostream& stream, const Velocity& v) {
    stream << "vx: " << v.x << ", vy: " << v.y << ", vz: " << v.z << ", vel: " << v.elevation << ", vaz: " << v.az;
    return stream;
}
#include "trajectory.h"




int main(int argc, char **argv) {
    YAML::Node config = YAML::LoadFile("/home/lee/unhuman/freebot-realtime/config/param.yaml");

    FreebotArmControl arm_control(config["arm"]["model"].as<std::string>(), config["arm"]["control_body"].as<std::string>());

    auto motor_names = config["motors"].as<std::vector<std::string>>();

    MotorPublisher<ArmStatus> pub("arm_status");
    MotorSubscriber<ArmCommand> sub("arm_command");
    MotorSubscriber<ArmCommandTrajectory> sub_trajectory("arm_command_trajectory");

    MotorManager m;
    m.get_motors_by_name(motor_names);


    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    int count = 0;
    ArmStatus status = {};
    Position &position = status.command.position;

    auto motor_status = m.read();
    // for (int i=0; i<motor_status.size(); i++) {
    //     Q[i] = motor_status[i].motor_position/100;
    //     if(i==0) Q[i] = -Q[i]+M_PI/2;
    //     std::cout << Q[i] << std::endl;
    // }
//    Q[3] = -M_PI/2;

    int last_command_num = 0;
    Trajectory trajectory;
    Position position_trajectory = position;
    Position position_sum = {};
    while(1) {
        count++;
        m.poll();
        auto motor_status = m.read();
        //std::cout << motor_status << std::endl;
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

        auto q = arm_control.step(position);

        for (int i=0; i<q.size(); i++) {
            status.command.joint_position[i] = q[i];
        }

        for (int i=0; i<motor_status.size(); i++) {
            status.measured.joint_position[i] = motor_status[i].motor_position/100;
            if(i==0) status.measured.joint_position[i] = -status.measured.joint_position[i];
        }

        std::vector<float> motor_desired(m.motors().size());
        for (int i=0; i<m.motors().size(); i++) {
            float tmp = status.command.joint_position[i];
            if (i==0) tmp = M_PI/2 - tmp;
            tmp *= 100;
            if (fabs(tmp - motor_status[i].motor_position) < 10) {
                motor_desired[i] = tmp;
            } else {
                motor_desired[i] = motor_status[i].motor_position;
            }
        }
        m.set_command_mode(POSITION);
        m.set_command_count(count);
        m.set_command_position(motor_desired);
        m.write_saved_commands();

        pub.publish(status);
        next_time += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next_time);
    }

    return 0;
}