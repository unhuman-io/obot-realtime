#include <motor_subscriber.h>
#include <thread>
#include <iostream>
#include <motor_manager.h>
#include <signal.h>

#include <yaml-cpp/yaml.h>
#include "trajectory.h"


bool run = true;

int main(int argc, char **argv) {
    if(argc != 2) {
        std::cout << "Usage freebot-test param.yaml" << std::endl;
        exit(1);
    }
    YAML::Node config = YAML::LoadFile(argv[1]);
    auto motor_names = config["motors"].as<std::vector<std::string>>();
    MotorManager m;
    m.get_motors_by_name(motor_names, true, true);
    std::cout << "Number of motors: " << m.motors().size() << std::endl;


    auto start_time = std::chrono::steady_clock::now();
    auto next_time = start_time;
    uint32_t count = 0;
    SimpleTrajectory trajectory, j2_trajectory;
    signal(SIGINT,[](int signum){run = 0;});
    std::vector<double> j3_traj {0, 10, -20, 20};
    std::vector<double> j2_traj {0, -20, 20, -20};
    int traj_point = 0;
    auto motor_status = m.read();
    trajectory.start(motor_status[4].motor_position, 0, j3_traj[0], 0, next_time);
    j2_trajectory.start(motor_status[3].motor_position, 0, j2_traj[0], 0, next_time);
    while(run) {
        count++;
        m.poll();
        auto motor_status = m.read();

        if (count % 4096 == 0) {
            int next_traj_point = traj_point + 1;
            if (next_traj_point >= j3_traj.size()) {
                next_traj_point = 0;
            }
            std::cout << "New trajectory point" << std::endl;
            // about 4 seconds start a new trajectory
            trajectory.start(j3_traj[traj_point],0,j3_traj[next_traj_point],0, next_time);
            j2_trajectory.start(j2_traj[traj_point],0,j2_traj[next_traj_point],0, next_time);
            traj_point = next_traj_point;
            std::cout << motor_status[3].motor_position << "\t" << motor_status[4].motor_position << std::endl;
        }
        std::vector<float> motor_desired(7,0);
        motor_desired[4] = trajectory.get_trajectory_position(next_time);
        motor_desired[3] = j2_trajectory.get_trajectory_position(next_time);
        std::vector<float> velocity_desired(7,0);
        velocity_desired[4] = trajectory.get_trajectory_velocity(next_time);
        velocity_desired[3] = j2_trajectory.get_trajectory_velocity(next_time);
        velocity_desired[0] = .1*traj_point;


        m.set_command_mode({VELOCITY, VELOCITY, POSITION, POSITION, POSITION, POSITION, POSITION});
        //m.set_command_mode(mode_command);
        m.set_command_count(count);
        m.set_command_position(motor_desired);
        m.set_command_velocity(velocity_desired);
        m.write_saved_commands();

        next_time += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next_time);
    }

    m.set_command_mode(OPEN);
    m.write_saved_commands();

    return 0;
}