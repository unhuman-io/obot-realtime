#pragma once
#include "freebot_messages.h"
#include "Eigen/Dense"
#include <string>
#include <motor_chain_messages.h>

class FreebotBaseControl {
 public:
    FreebotBaseControl() {}
    
    // Input BaseCommand and MotorChainStatus, returns BaseStatus and MotorChainCommand
    void step(const BaseCommand &c, const MotorChainStatus &ms, 
      BaseStatus *s, MotorChainCommand *mc);

 private:

};
