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

        void update(json sensor_fusion, double cur_car_s, vector<double> maps_x, vector<double> maps_y);

        vector<Vehicle *> get_vehicles();

        vector<Vehicle *> get_vehicles_in_lane(int lane);

        Vehicle* get_closest_vehicle_ahead_of(Vehicle *vehicle);

        Vehicle* get_closest_vehicle_ahead_of(Vehicle *vehicle, int lane);

        double get_average_lane_speed_ahead_of(Vehicle *vehicle, int lane);

        vector<double> get_average_lane_speeds_ahead_of(Vehicle* cur_vehicle);

        vector<double> get_speed_of_closest_vehicles_for(Vehicle* cur_vehicle);

        bool has_collisions(Trajectory trajectory, double collision_distance, int lane);

        double empty_lane_speed = std::numeric_limits<double>::max();

        int lanes = 3;

    private:
        map<int, Vehicle*> vehicles;

        double sensor_range = 100.0;

        double closest_range = 20.0;
    };
}

#endif //CARND_PATH_PLANNING_ROAD_H
