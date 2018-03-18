//
// Created by Дима on 13.03.2018.
//

#ifndef CARND_PATH_PLANNING_VEHICLE_H
#define CARND_PATH_PLANNING_VEHICLE_H

#include <chrono>
#include <vector>
#include <math.h>

#include "trajectory.h"
#include "utils.h"

namespace car_nd_path_planning {
    using namespace std;
    using namespace std::chrono;

    class Vehicle {

    public:

        Vehicle(double x, double y, double yaw, double s, double d);

        Vehicle(int id, double x, double y, double vx, double vy, double s, double d);

        void init(int id, double x, double y, double vx, double vy, double s, double d);

        void update(double x, double y, double yaw, double s, double d, vector<double> maps_x, vector<double> maps_y);

        void update(double x, double y, double vx, double vy, double s, double d, vector<double> maps_x, vector<double> maps_y);

        Trajectory predict_trajectory(int horizon, double delta_t);

        bool has_collisions(Trajectory trajectory, double collision_distance);

        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;

        double yaw;

        double speed;
        int lane;
        double acceleration_x;
        double acceleration_y;

        milliseconds last_update_time;

        vector<double> maps_x;
        vector<double> maps_y;

    private:
        int default_car_id = -1;

    };

}

#endif //CARND_PATH_PLANNING_VEHICLE_H
