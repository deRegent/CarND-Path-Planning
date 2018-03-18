//
// Created by Дима on 17.03.2018.
//

#ifndef CARND_PATH_PLANNING_BEHAVIOR_H
#define CARND_PATH_PLANNING_BEHAVIOR_H

#include <math.h>
#include "json.hpp"
#include <map>
#include <vector>
#include <chrono>

#include "vehicle.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_builder.h"

namespace car_nd_path_planning {
    using namespace std;

    enum class State {
        KeepLane = 0,
        PrepareLaneChangeLeft = 1,
        PrepareLaneChangeRight = 2,
        LaneChangeLeft = 3,
        LaneChangeRight = 4
    };

    class Behavior {

    public:
        Behavior(Vehicle* cur_vehicle, Road* road);

        void update(vector<double> previous_path_x, vector<double> previous_path_y,
                    vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                    vector<double> map_waypoints_s);

        double get_ref_velocity();

        int get_target_lane();

    private:
        State state = State::KeepLane;

        Vehicle* cur_vehicle;
        Road* road;
        int target_lane;
        double ref_velocity = 0.0;

        double speed_limit = 49.5;
        double target_speed = speed_limit;
        double sensor_range = 400.0;
        double min_safe_distance_threshold = 15.0;
        double collision_threshold = 10.0;
        double velocity_change = 0.224;

        vector<double> previous_path_x;
        vector<double> previous_path_y;
        vector<double> map_waypoints_x;
        vector<double> map_waypoints_y;
        vector<double> map_waypoints_s;

        milliseconds observation_time;
        long last_lane_change_millisec = 0;
        long lane_transition_millisec = 1000;

        void updateState();
        void updateParams();

        void follow_closest_vehicle();

        void evaluate_keep_lane_trajectory();

        bool has_collision_on_lane_change(int lane);
    };
}

#endif //CARND_PATH_PLANNING_BEHAVIOR_H
