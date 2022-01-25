#include <motor_subscriber.h>
#include <motor_publisher.h>
#include <thread>
#include <iostream>
#include "obot_messages.h"
#include <motor_manager.h>
#include <signal.h>

#include <yaml-cpp/yaml.h>
#include "obot_arm_control.h"
#include "obot_base_control.h"
#include "motor_app.h"
#include "motor_thread.h"
#include <motor_chain_messages.h>

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

static Eigen::VectorXd gear_ratio;

class Task : public MotorThread {
 public:
  Task(YAML::Node &config) : MotorThread(config["frequency"].as<int>()) {
  }
 protected:
	virtual void post_init() {

	}
	virtual void pre_update() {
        base_command_ = base_sub_.read();
	}

    virtual void controller_update() {
        count_++;
        MotorStatus *motor_status = data_.statuses;
        auto &motor_commands = motor_manager_.commands();
        MotorChainStatus base_motor_status(motor_status, 2);
        MotorChainCommand base_motor_command(motor_commands.data(), 2);

        base_control_.step(base_command_, base_motor_status, &base_status_, &base_motor_command);
        base_status_.measured.positionl = motor_status[0].motor_position;
        base_status_.measured.positionr = motor_status[1].motor_position;

        motor_manager_.set_command_count(count_);
    }

    virtual void post_update() {
        base_pub_.publish(base_status_);
    }

    virtual void finish() {
        motor_manager_.set_command_mode(OPEN);
        motor_manager_.write_saved_commands();
    }

 private:
    MotorPublisher<BaseStatus> base_pub_{"base_status"};
    MotorSubscriber<BaseCommand> base_sub_{"base_command"};
    BaseStatus base_status_ = {};
    BaseCommand base_command_ = {};
    ObotBaseControl base_control_;

    int count_ = 0;
};

class MotorAppObot : public MotorApp {
 public:
    MotorAppObot(int argc, char **argv, MotorThread *motor_thread, YAML::Node &config) : 
        config_(config),
        MotorApp(argc, argv, motor_thread) {}
    virtual void select_motors(MotorManager *m) {
        auto motor_names = config_["motors"].as<std::vector<std::string>>();   
        m->get_motors_by_name(motor_names, true, true);
        std::cout << m->motors().size() << std::endl;
        // if motors are simulated set the gear ratio
        for (int i=0;i<m->motors().size();i++) {
            auto mot = m->motors()[i];
            if (typeid(*(mot.get())) == typeid(SimulatedMotor)) {
                dynamic_cast<SimulatedMotor*>(mot.get())->set_gear_ratio(gear_ratio[i]);
            }
        }
    }
 private:
    YAML::Node &config_;
};

int main (int argc, char **argv)
{	
    if (argc != 2) {
        std::cout << "Usage " << argv[0] << " PARAM.yaml" << std::endl;
        return 1;
    }
    YAML::Node config = YAML::LoadFile(argv[1]);
    gear_ratio = read_yaml_vector(config["gear_ratio"]);
	Task task(config);
	auto app = MotorAppObot(argc, argv, &task, config);
	return app.run();
}
