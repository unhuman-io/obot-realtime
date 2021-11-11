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
#include "motor_app.h"
#include "motor_thread.h"

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

static YAML::Node config;
static Eigen::VectorXd gear_ratio;

class Task : public MotorThread {
 public:
  Task(int frequency) : MotorThread(frequency) {
  }
 protected:
	virtual void post_init() {

	}
	virtual void pre_update() {
        base_command_ = base_sub_.read();
	}

    virtual void controller_update() {
        count_++;
        std::vector<MotorStatus> motor_status(data_.statuses, data_.statuses + motor_manager_.motors().size());

        Eigen::VectorXd q_base = Eigen::VectorXd::Zero(2);
        Eigen::VectorXd qd_base = base_control_.step(base_command_.x, base_command_.az);

        std::vector<uint8_t> mode_command(2, VELOCITY);

        Eigen::VectorXd q(2), qd(2);
        q << q_base;
        qd << qd_base;

        Eigen::VectorXd qm(2), qmd(2);
        qm = q.cwiseProduct(gear_ratio.block(0,0,1,2));
        qmd = qd.cwiseProduct(gear_ratio.block(0,0,1,2));

        base_status_.measured.positionl = motor_status[0].motor_position;
        base_status_.measured.positionr = motor_status[1].motor_position;
        
        std::vector<float> motor_desired(qm.data(), qm.data() + qm.size());
        std::vector<float> velocity_desired(qmd.data(), qmd.data() + qmd.size());

        motor_manager_.set_command_mode(mode_command);
        motor_manager_.set_command_count(count_);
        motor_manager_.set_command_position(motor_desired);
        motor_manager_.set_command_velocity(velocity_desired);
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
    FreebotBaseControl base_control_;

    int count_ = 0;
};

class MotorAppFreebot : public MotorApp {
 public:
    MotorAppFreebot(int argc, char **argv, MotorThread *motor_thread) : 
        MotorApp(argc, argv, motor_thread) {}
    virtual void select_motors(MotorManager *m) {
        auto motor_names = config["motors"].as<std::vector<std::string>>();   
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
};

int main (int argc, char **argv)
{	
    config = YAML::LoadFile("/home/lee/freebot/freebot-realtime/config/param.yaml");
    gear_ratio = read_yaml_vector(config["gear_ratio"]);
	Task task(1000);
	auto app = MotorAppFreebot(argc, argv, &task);
	return app.run();
}
