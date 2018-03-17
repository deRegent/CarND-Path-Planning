//
// Created by Дима on 17.03.2018.
//

#ifndef CARND_PATH_PLANNING_UTILS_H
#define CARND_PATH_PLANNING_UTILS_H

#include <math.h>

namespace car_nd_path_planning {
    using namespace std;

    constexpr double pi() { return M_PI; }

    double deg2rad(double x) { return x * pi() / 180; }

    double rad2deg(double x) { return x * 180 / pi(); }

    double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
}

#endif //CARND_PATH_PLANNING_UTILS_H
