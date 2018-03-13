//
// Created by Дима on 13.03.2018.
//

#ifndef CARND_PATH_PLANNING_VEHICLE_H
#define CARND_PATH_PLANNING_VEHICLE_H

#include <chrono>
#include <vector>
#include "trajectory.h"

namespace car_nd_path_planning {

    class Vehicle {

    public:

        Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

        void update(double x, double y, double vx, double vy, double s, double d);

        Trajectory predict_trajectory(double delta_t);

    private:
        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;

        double speed;
        int lane;
        double acceleration_x;
        double acceleration_y;

        milliseconds last_update_time;

    };

}

#endif //CARND_PATH_PLANNING_VEHICLE_H
