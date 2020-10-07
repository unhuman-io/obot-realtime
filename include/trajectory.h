#pragma once
#include "freebot_messages.h"
#include <chrono>
#include <limits>
#include <algorithm>
#include <cmath>


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
        if (s > 0) {
            for (double q : {-1, 1}) {
                double t1 = (-2*v0 + q*std::sqrt(s))*.5*a;
                if (t1 > 0) {
                    double t3 = 2*t1 - dv/a;
                    if (t3 > 0) {
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
        for (int i=0; i<trajectory.num_points; i++) {
            trajectory_coeffs_[i] = calculate_coeffs(tmp_position, tmp_velocity, trajectory.trajectory_point[i]);
            tmp_position = trajectory.trajectory_point[i].position;
            tmp_velocity = trajectory.trajectory_point[i].velocity;
        }
    }
    Position get_trajectory_position(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        TrajectoryCoeffs &c = trajectory_coeffs_[trajectory_coeff_index(time)];        
        auto t = trajectory_index_time(time);
        Position p = {};
        auto s = get_trajectory_values(c, t);
        //p.x = s.x;
        return p;
    }
    Velocity get_trajectory_velocity(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        Velocity v = {};
        return v;
    }
    // find which trajectory segment that time is in
    int trajectory_coeff_index(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        return 0;
    }
    double trajectory_time(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        std::chrono::duration<double> seconds = time - time_start_;
        return seconds.count();
    }
    double trajectory_index_time(const std::chrono::time_point<std::chrono::steady_clock> &time) {
        return trajectory_time(time) - trajectory_time_points_[trajectory_coeff_index(time)];
    }
 private:
    TrajectoryCoeffs trajectory_coeffs_[POSITION_TRAJECTORY_MAX_POINTS];
    double trajectory_time_points_[POSITION_TRAJECTORY_MAX_POINTS];

    // calculates a trapezoidal trajectory
    TrajectoryCoeffs calculate_coeffs(const Position &position, const Velocity &velocity, const PositionTrajectory::TrajectoryPoint &trajectory_point) {
        TrajectoryCoeffs c;
        // first calculate a pyramid velocity function. If it exceeds max velocity, then switch to trapezoidal

        c.a = 1;
    }

    std::chrono::time_point<std::chrono::steady_clock> time_start_;
};