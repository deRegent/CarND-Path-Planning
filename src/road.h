//
// Created by Дима on 15.03.2018.
//

#ifndef CARND_PATH_PLANNING_ROAD_H
#define CARND_PATH_PLANNING_ROAD_H

#include <math.h>
#include "json.hpp"
#include <map>
#include <vector>

#include "vehicle.h"
#include "trajectory.h"

namespace car_nd_path_planning {

    using namespace std;
    using json = nlohmann::json;

    class Road {

    public:
        Road();

        void update(json sensor_fusion, double cur_car_s);

        vector<Vehicle *> get_vehicles();

        vector<Vehicle *> get_vehicles_in_lane(int lane);

        Vehicle* get_closest_vehicle_ahead_of(Vehicle *vehicle);

        Vehicle* get_closest_vehicle_ahead_of(Vehicle *vehicle, int lane);

        vector<double> get_average_lane_speed(int lane);

        vector<double> get_average_lane_speeds();

        vector<double> get_speed_of_closest_vehicles_for(Vehicle* cur_vehicle);

        bool has_collisions(Trajectory trajectory, double collision_distance);

    private:
        map<int, Vehicle*> vehicles;

        int lanes = 3;

        double sensor_range = 400.0;

        double empty_lane_speed = std::numeric_limits<double>::max();
    };
}

#endif //CARND_PATH_PLANNING_ROAD_H
