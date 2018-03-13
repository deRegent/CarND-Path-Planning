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
        Trajectory(vector<double> ptsx, vector<double> ptsy);

        vector<double> path_x;
        vector<double> path_y;
    };

}

#endif //CARND_PATH_PLANNING_TRAJECTORY_H
