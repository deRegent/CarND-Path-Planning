//
// Created by Дима on 13.03.2018.
//

#ifndef CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include "spline.h"
#include <math.h>
#include <vector>

#include "trajectory.h"
#include "utils.h"

namespace car_nd_path_planning {
    using namespace std;

    class TrajectoryBuilder {
    public:
        TrajectoryBuilder();

        Trajectory build_trajectory(double car_x, double car_y, double car_s,
                                    double car_yaw, int target_lane, double ref_velocity,
                                    vector<double> previous_path_x, vector<double> previous_path_y,
                                    vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                                    vector<double> map_waypoints_s);

        int horizon = 50;
        double delta_t = 0.02;

    };

}

#endif //CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H
