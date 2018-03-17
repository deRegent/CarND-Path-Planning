//
// Created by Дима on 13.03.2018.
//

#include "trajectory_builder.h"

namespace car_nd_path_planning {
    using namespace std;

    TrajectoryBuilder::TrajectoryBuilder() { }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
        int prev_wp = -1;

        while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
            prev_wp++;
        }

        int wp2 = (prev_wp + 1) % maps_x.size();

        double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s - maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
        double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

        double perp_heading = heading - pi() / 2;

        double x = seg_x + d * cos(perp_heading);
        double y = seg_y + d * sin(perp_heading);

        return {x, y};

    }

    Trajectory TrajectoryBuilder::build_trajectory(double car_x, double car_y, double car_s,
                                                   double car_yaw, int target_lane, double ref_velocity,
                                                   vector<double> previous_path_x, vector<double> previous_path_y,
                                                   vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                                                   vector<double> map_waypoints_s) {

        // --------------- use code provided by Udacity in project's walkthrough ---------------
        // --------------- to control trajectory with spline library             ---------------

        int prev_size = previous_path_x.size();

        vector<double> ptsx;
        vector<double> ptsy;

        double ref_x = car_x;
        double ref_y = car_y;
        double ref_yaw = deg2rad(car_yaw);

        if (prev_size < 2) {
            ptsx.push_back(car_x - cos(car_yaw));
            ptsy.push_back(car_y - sin(car_yaw));

            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
        }
        else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);

            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
        }

        vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x,
                                        map_waypoints_y);
        vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x,
                                        map_waypoints_y);
        vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x,
                                        map_waypoints_y);

        ptsx.push_back(next_wp0[0]);
        ptsy.push_back(next_wp0[1]);

        ptsx.push_back(next_wp1[0]);
        ptsy.push_back(next_wp1[1]);

        ptsx.push_back(next_wp2[0]);
        ptsy.push_back(next_wp2[1]);

        for (int i = 0; i < ptsx.size(); i++) {
            double new_x = ptsx[i] - ref_x;
            double new_y = ptsy[i] - ref_y;

            ptsx[i] = (new_x * cos(0 - ref_yaw)) - (new_y * sin(0 - ref_yaw));
            ptsy[i] = (new_x * sin(0 - ref_yaw)) + (new_y * cos(0 - ref_yaw));

        }

        tk::spline s;

        s.set_points(ptsx, ptsy);

        vector<double> next_x_vals;
        vector<double> next_y_vals;

        for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        double target_x = 30.0;
        double target_y = s(target_x);

        double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
        double N = target_dist / (0.02 * ref_velocity / 2.24);
        double delta_x = target_x / N;

        double new_x = 0.0;
        double new_y = 0.0;

        for (int i = 0; i < (50 - prev_size); i++) {
            new_x = new_x + delta_x;
            new_y = s(new_x);

            double out_x = ref_x + (new_x * cos(ref_yaw)) - (new_y * sin(ref_yaw));
            double out_y = ref_y + (new_x * sin(ref_yaw)) + (new_y * cos(ref_yaw));

            next_x_vals.push_back(out_x);
            next_y_vals.push_back(out_y);
        }

        Trajectory trajectory(next_x_vals, next_y_vals, this->horizon, this->delta_t);

        return trajectory;
    }
}

