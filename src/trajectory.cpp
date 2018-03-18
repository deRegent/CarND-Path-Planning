//
// Created by Дима on 13.03.2018.
//

#include "trajectory.h"

namespace car_nd_path_planning {
    using namespace std;

    Trajectory::Trajectory(vector<double> ptsx, vector<double> ptsy, int horizon, double delta_t) {

        this->path_x = ptsx;
        this->path_y = ptsy;

        this->horizon = horizon;
        this->delta_t = delta_t;

        this->ref_yaw = ref_yaw;
    }

}
