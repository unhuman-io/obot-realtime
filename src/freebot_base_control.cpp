#include "freebot_base_control.h"

void FreebotBaseControl::step(const BaseCommand &c, const MotorChainStatus &ms, 
    BaseStatus *s, MotorChainCommand *mc) {
    Eigen::Vector2d out;
    out << c.x-c.az, c.x+c.az;
    s->command.wl = out[0];
    s->command.wr = out[1];
    mc->velocity_desired() = out.cast<float>();
    mc->mode_desired().array() = VELOCITY;
}
