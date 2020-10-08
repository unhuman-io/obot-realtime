#pragma once
#include "freebot_messages.h"
#include <chrono>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>


double sign(double d) {
    if (d >= 0) {
        return 1;
    }
    return -1;
}

struct TrajectoryCoeffs {
    double a, t1, t2, t3, p0, v0;
};

struct TrajectoryState {
    double x, v;
};

TrajectoryCoeffs calc_trapezoidal_coeffs(double p0, double p3, double amax, double vmax=std::numeric_limits<double>::infinity(), double v0=0, double v3=0) {
    TrajectoryCoeffs c;
    v0 = std::min(std::max(v0,-vmax), vmax);
    v3 = std::min(std::max(v3,-vmax), vmax);
    double dp = p3-p0;
    double dv = v3-v0;
    c.v0 = v0;
    c.p0 = p0;
    double t3sol = std::numeric_limits<double>::infinity();
    for (auto a : {amax, -amax} ) {
        double s = 4*v0*v0 + 4*(dp*a + .5*(v0+v3)*dv);
        if (s >= 0) {
            for (double q : {-1, 1}) {
                double t1 = (-2*v0 + q*std::sqrt(s))*.5*a;
                if (t1 >= 0) {
                    double t3 = 2*t1 - dv/a;
                    if (t3 >= 0) {
                        if (t3 < t3sol) {
                            t3sol = t3;
                            c.t1 = t1;
                            c.t3 = t3;
                            c.a = a;
                        }
                    }
                }
            }
        }
    }

    if (std::abs(c.v0+c.a*c.t1) < vmax) {
        // pyramid velocity
        c.t2 = c.t1;
    } else {
        // trapezoid velocity
        double q = sign(c.v0+c.a*c.t1);
        double vsat = q*vmax;
        double tp = c.t1;
        c.t1 = (vsat - v0)/c.a;
        double xtrap = 2*(tp-c.t1)*(vsat+.5*c.a*(tp-c.t1));
        c.t2 = c.t1 + xtrap/vsat;
        c.t3 = c.t3 - 2*tp + c.t1 + c.t2;
    }
    std::cout << c.t1 << std::endl;
    return c;
}

TrajectoryState get_trajectory_values(const TrajectoryCoeffs &c, double t) {
    TrajectoryState s;
    if (t > c.t3) {
        t = c.t3;
    }
    if (t < c.t1) {
        s.x = c.p0 + c.v0*t + .5*c.a*t*t;
        s.v = c.v0 + c.a*t;
    } else if (t < c.t2) {
        s.x = c.p0 + c.v0*c.t1 + .5*c.a*c.t1*c.t1 + (c.v0 + c.a*c.t1)*(t-c.t1);
        s.v = c.v0 + c.a*c.t1;
    } else {
        s.x = c.p0 + c.v0*c.t1 + .5*c.a*c.t1*c.t1 + (c.v0 + c.a*c.t1)*(t-c.t1) - .5*c.a*(t-c.t2)*(t-c.t2);
        s.v = c.v0 + c.a*c.t1 - c.a*(t-c.t2);
    }
    return s;
}

class Trajectory {
 public:
    Trajectory() {}
    void start(const PositionTrajectory &trajectory, const Position &current_position, const Velocity &current_velocity, const std::chrono::time_point<std::chrono::steady_clock> &time) {
        time_start_ = time;
        Position tmp_position = current_position;
        Velocity tmp_velocity = current_velocity;
        num_points_ = trajectory.num_points;
        for (int i=0; i<num_points_; i++) {
            trajectory_coeffs_[i] = calculate_coeffs(tmp_position, tmp_velocity, trajectory.trajectory_point[i]);
            tmp_position = trajectory.trajectory_point[i].position;
            tmp_velocity = trajectory.trajectory_point[i].velocity;
            std::cout << trajectory.trajectory_point[i].position << std::endl;
        }
        for (int i=1; i<num_points_; i++) {
            trajectory_coeffs_[i].t_start = trajectory_coeffs_[i-1].t_start + trajectory_coeffs_[i-1].t3_max;
        }
    }
    Position get_trajectory_position(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        TrajectoryCoeffStruct &c = trajectory_coeffs_[trajectory_coeff_index(time)];        
        auto t = trajectory_index_time(time);
        //std::cout << "t: " << t << ", ";
        Position p = {};
        auto s = get_trajectory_values(c.x, t);
        p.x = s.x;
        s = get_trajectory_values(c.y, t);
        p.y = s.x;
        s = get_trajectory_values(c.z, t);
        p.z = s.x;
        s = get_trajectory_values(c.elevation, t);
        p.elevation = s.x;
        s = get_trajectory_values(c.az, t);
        p.az = s.x;
        return p;
    }
    Velocity get_trajectory_velocity(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        Velocity v = {};
        return v;
    }
    // find which trajectory segment that time is in
    int trajectory_coeff_index(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        int index = 0;
        double t3_sum = 0;
        for (int i=0; i<num_points_; i++) {
            if (trajectory_time(time) > t3_sum) {
                t3_sum += trajectory_coeffs_[i].t3_max;
                index = i;
            }
        }
        return index;
    }
    double trajectory_time(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        std::chrono::duration<double> seconds = time - time_start_;
        return seconds.count();
    }
    double trajectory_index_time(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        return trajectory_time(time) - trajectory_coeffs_[trajectory_coeff_index(time)].t_start;
    }
 private:
    struct TrajectoryCoeffStruct {
        TrajectoryCoeffs x, y, z, elevation, az;
        double t3_max;
        double t_start;
    } trajectory_coeffs_[POSITION_TRAJECTORY_MAX_POINTS] = {};
    int num_points_ = 0;
    // calculates a trapezoidal trajectory
    TrajectoryCoeffStruct calculate_coeffs(const Position &position, const Velocity &velocity, const PositionTrajectory::TrajectoryPoint &trajectory_point) {
        TrajectoryCoeffStruct s;
        s.x = calc_trapezoidal_coeffs(position.x, trajectory_point.position.x, 1);
        s.y = calc_trapezoidal_coeffs(position.y, trajectory_point.position.y, 1);
        s.z = calc_trapezoidal_coeffs(position.z, trajectory_point.position.z, 1);
        s.elevation = calc_trapezoidal_coeffs(position.elevation, trajectory_point.position.elevation, 1);
        s.az = calc_trapezoidal_coeffs(position.az, trajectory_point.position.az, 1);
        s.t3_max = std::max(s.x.t3, std::max(s.y.t3, std::max(s.z.t3, std::max(s.elevation.t3, s.az.t3))));
        s.t3_max = std::max(s.x.t3, std::max(s.y.t3, s.z.t3));
        return s;
    }

    std::chrono::time_point<std::chrono::steady_clock> time_start_;
};