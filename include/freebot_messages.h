#pragma once

typedef struct {
    double x,y,z;
    double ax,ay,az;
} Velocity;

typedef struct {
    double x,y,z;
    double ax,ay,az;
} Position;

typedef double ArmJointPosition[5];

struct ArmCommand {
    Velocity velocity;
};

struct ArmStatus {
    struct {
        Position position;
        ArmJointPosition joint_position;
    } command;
};
