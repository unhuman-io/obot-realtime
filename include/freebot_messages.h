#pragma once

#define POSITION_TRAJECTORY_MAX_POINTS  10

typedef struct {
    double x,y,z;
    double elevation, az;
} Velocity;

typedef struct {
    double x,y,z;
    double elevation, az;
} Position;

typedef double ArmJointPosition[5];

typedef struct {
    int num_points;
    struct TrajectoryPoint {
        // double seconds_after_start;
        Position position;
        Velocity velocity;
    } trajectory_point[POSITION_TRAJECTORY_MAX_POINTS];
} PositionTrajectory;

struct ArmCommand {
    Velocity velocity;
};

struct ArmCommandTrajectory {
    int command_num;
    PositionTrajectory position_trajectory;
};

struct ArmStatus {
    struct {
        Position position;
        ArmJointPosition joint_position;
    } command;
    struct {
        ArmJointPosition joint_position;
    } measured;
};

struct BaseCommand {
    double x, az;
};

struct BaseStatus {
    struct {
       double wl, wr;
    } command;
    struct {
       double positionl, positionr;
    } measured;
};
