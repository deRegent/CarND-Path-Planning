//
// Created by Дима on 13.03.2018.
//

#ifndef CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H

#include "trajectory.h"
#include "spline.h"
#include <math.h>

namespace car_nd_path_planning {
    using namespace std;

    class TrajectoryBuilder {
    public:
        TrajectoryBuilder();

        Trajectory build_trajectory(double car_x, double car_y, double car_s,
                              double car_yaw, int target_lane, double ref_velocity);

    };

}

#endif //CARND_PATH_PLANNING_TRAJECTORY_GENERATOR_H
