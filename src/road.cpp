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

            double distance = std::abs(s - cur_car_s);

            if (distance <= this->sensor_range) {
                // car is in the sensor range
                if (this->vehicles.find(id) == vehicles.end()) {
                    Vehicle *vehicle = new Vehicle(id, x, y, vx, vy, s, d);
                    this->vehicles[id] = vehicle;
                } else {
                    Vehicle *vehicle = vehicles[id];
                    vehicle->update(x, y, vx, vy, s, d);
                }
            } else {
                this->vehicles.erase(id);
            }
        }
    }

    vector<Vehicle*> Road::get_vehicles() {
        vector<Vehicle *> vehicles;

        for (const auto &item : this->vehicles) {
            int id = item.first;
            Vehicle *road_vehicle = item.second;

            vehicles.push_back(road_vehicle);
        }

        return vehicles;
    }

    vector<Vehicle*> Road::get_vehicles_in_lane(int lane) {
        vector<Vehicle *> vehicles;

        for (const auto &item : this->vehicles) {
            int id = item.first;
            Vehicle *road_vehicle = item.second;

            if (lane != road_vehicle->lane) {
                // skip cars not in the target lane
                continue;
            }

            vehicles.push_back(road_vehicle);
        }

        return vehicles;
    }

    Vehicle* Road::get_closest_vehicle_ahead_of(Vehicle *vehicle) {
        return this->get_closest_vehicle_ahead_of(vehicle, vehicle->lane);
    }

    Vehicle* Road::get_closest_vehicle_ahead_of(Vehicle *vehicle, int lane) {

        double min_distance = -1;
        Vehicle *target_vehicle = NULL;

        for (const auto &item : this->vehicles) {
            int id = item.first;
            Vehicle *road_vehicle = item.second;

            if (lane != road_vehicle->lane) {
                // skip cars not in the target lane
                continue;
            }

            double distance = std::abs(vehicle->s - road_vehicle->s);

            bool is_ahead = (road_vehicle->s - vehicle->s) > 0;

            if (is_ahead && (min_distance < 0 || distance < min_distance)) {
                min_distance = distance;
                target_vehicle = road_vehicle;
            }
        }

        return target_vehicle;
    }

    double Road::get_average_lane_speed(int lane){
        double total_speed = 0;

        vector<Vehicle *> vehicles = this->get_vehicles_in_lane(lane);

        for (int i=0; i < vehicles.size(); i++){
            Vehicle* vehicle = vehicles[i];
            total_speed += vehicle->speed;
        }

        if (total_speed == 0){
            return this->empty_lane_speed;
        }

        return total_speed / vehicles.size();
    }

    vector<double> Road::get_average_lane_speeds() {
        vector<double> speeds;

        for (int lane = 0; lane < this->lanes; lane++) {
            speeds.push_back(this->get_average_lane_speed(lane));
        }

        return speeds;
    }

    vector<double> Road::get_speed_of_closest_vehicles_for(Vehicle *cur_vehicle) {
        vector<double> speeds;

        for (int lane = 0; lane < this->lanes; lane++) {
            Vehicle *vehicle = this->get_closest_vehicle_ahead_of(cur_vehicle, lane);

            if (vehicle == NULL) {
                speeds.push_back(this->empty_lane_speed);
            } else {
                speeds.push_back(vehicle->speed);
            }
        }

        return speeds;
    }

    bool Road::has_collisions(Trajectory trajectory, double collision_distance){
        vector<Vehicle *> vehicles = this->get_vehicles();

        bool hasCollisions = false;

//        printf(" | Has collisions with vehicles: ");
        for (int i=0; i < vehicles.size(); i++){
            Vehicle* vehicle = vehicles[i];

            if (vehicle->has_collisions(trajectory, collision_distance)){
//                printf(" lane: %d, ", vehicle->lane);
                hasCollisions = true;
            }
        }

//        printf(" | ");

        return hasCollisions;
    }

}