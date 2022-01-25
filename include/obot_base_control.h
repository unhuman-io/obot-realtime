#pragma once
#include "obot_messages.h"
#include "Eigen/Dense"
#include <string>
#include <motor_chain_messages.h>

class ObotBaseControl {
 public:
    ObotBaseControl() {}
    
    // Input BaseCommand and MotorChainStatus, returns BaseStatus and MotorChainCommand
    void step(const BaseCommand &c, const MotorChainStatus &ms, 
      BaseStatus *s, MotorChainCommand *mc);

 private:

};
