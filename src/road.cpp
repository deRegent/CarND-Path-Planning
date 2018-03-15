//
// Created by Дима on 15.03.2018.
//

#include "road.h"

namespace car_nd_path_planning {
    using namespace std;

    Road::Road() {

    }

    void Road::update(json sensor_fusion, double cur_car_s) {
        for (int i = 0; i < sensor_fusion.size(); i++) {

            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];

            distance = Math.abs(s - cur_car_s);

            if (distance <= this->sensor_range) {
                // car is in the sensor range
                if (this->vehicles.find(id) == vehicles.end()) {
                    Vehicle *vehicle = new Vehicle(id, x, y, vx, vy, s, d);
                    this->vehicles[id] = vehicle;
                } else {
                    vehicle = vehicles[id];
                    (*vehicle).update(x, y, vx, vy, s, d);
                }
            } else {
                vehicles.erase(id);
            }
        }
    }

    Vehicle *Road::get_closest_vehicle_ahead_of(Vehicle *vehicle) {

        double min_distance = -1;
        Vehicle *target_vehicle = NULL;

        for (const auto &item : myMap) {
            id = item.first;
            Vehicle *road_vehicle = item.first;

            if (vehicle->lane != road_vehicle->lane) {
                // skip cars not in the same lane
                continue;
            }

            double distance = Math.abs(vehicle->s - road_vehicle->s);

            bool is_ahead = (road_vehicle->s - vehicle->s) > 0;

            if (is_ahead && (min_distance < 0 || distance < min_distance)) {
                min_distance = car->distance;
                target_vehicle = road_vehicle;
            }
        }

        return target_vehicle;
    }

}