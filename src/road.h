//
// Created by Дима on 15.03.2018.
//

#ifndef CARND_PATH_PLANNING_ROAD_H
#define CARND_PATH_PLANNING_ROAD_H

#include <math.h>
#include "json.hpp"
#include <map>

#include "vehicle.h"

namespace car_nd_path_planning {

    using namespace std;
    using json = nlohmann::json;

    class Road {

    public:
        Road();

        void update(json sensor_fusion, double cur_car_s);

        Vehicle* get_closest_vehicle_ahead_of(Vehicle* vehicle);

    private:
        map<int, Vehicle*> vehicles;

        double sensor_range = 400.0;
    };
}

#endif //CARND_PATH_PLANNING_ROAD_H
