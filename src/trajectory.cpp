//
// Created by Дима on 13.03.2018.
//

#include "trajectory.h"

namespace car_nd_path_planning {
    using namespace std;

    Trajectory::Trajectory(vector<double> ptsx, vector<double> ptsy) {

        this->path_x = ptsx;
        this->path_y = ptsy;

    }

}
