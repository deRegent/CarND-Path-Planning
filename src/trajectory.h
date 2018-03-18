//
// Created by Дима on 13.03.2018.
//

#ifndef CARND_PATH_PLANNING_TRAJECTORY_H
#define CARND_PATH_PLANNING_TRAJECTORY_H

#include <vector>

namespace car_nd_path_planning {
    using namespace std;

    class Trajectory {
    public:
        Trajectory(vector<double> ptsx, vector<double> ptsy, double ref_yaw, int horizon, double delta_t);

        vector<double> path_x;
        vector<double> path_y;

        int horizon;
        double delta_t;

        double ref_yaw;

    };

}

#endif //CARND_PATH_PLANNING_TRAJECTORY_H