//
// Created by Дима on 17.03.2018.
//

#include "behavior.h"

namespace car_nd_path_planning {
    using namespace std;

    Behavior::Behavior(Vehicle *cur_vehicle, Road *road) {
        this->cur_vehicle = cur_vehicle;
        this->road = road;
        this->target_lane = cur_vehicle->lane;
    }

    void Behavior::update(vector<double> previous_path_x, vector<double> previous_path_y,
                          vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                          vector<double> map_waypoints_s) {

        this->updateState(previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s);
        this->updateParams(previous_path_x.size());
    }

    void Behavior::updateState(vector<double> previous_path_x, vector<double> previous_path_y,
                               vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                               vector<double> map_waypoints_s) {

        if (this->state == State::KeepLane) {

            this->evaluate_keep_lane_trajectory();

        } else if (this->state == State::PrepareLaneChangeRight || this->state == State::PrepareLaneChangeLeft) {

            this->evaluate_keep_lane_trajectory();

            if (this->state == State::PrepareLaneChangeRight) {
                trajectory_lane = this->cur_vehicle->lane + 1;
            } else if (this->state == State::PrepareLaneChangeLeft) {
                trajectory_lane = this->cur_vehicle->lane - 1;
            } else {
                return;
            }

            TrajectoryBuilder trajectoryBuilder;

            int trajectory_lane;

            Trajectory trajectory = trajectoryBuilder.build_trajectory(this->cur_vehicle->x,
                                                                       this->cur_vehicle->y,
                                                                       this->cur_vehicle->s,
                                                                       this->cur_vehicle->yaw,
                                                                       trajectory_lane,
                                                                       this->ref_velocity,
                                                                       previous_path_x,
                                                                       previous_path_y,
                                                                       map_waypoints_x,
                                                                       map_waypoints_y,
                                                                       map_waypoints_s);

            bool has_collisions = road->has_collisions(trajectory, this->collision_threshold);

            if (!has_collisions) {
                printf(" | State: Ready to change lane Right! | ");
                if (this->state == State::PrepareLaneChangeRight) {
                    printf(" | State: Ready to change lane Right! | ");
//                    this->state = State::LaneChangeRight;
                } else {
                    printf(" | State: Ready to change lane Left! | ");
//                    this->state = State::LaneChangeLeft;
                }
            }
        } else if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {
            if (this->cur_vehicle->lane == this->target_lane) {
                this->state = State::KeepLane;
            }
        }
    }

    void Behavior::updateParams(int prev_size) {

        if (this->state == State::KeepLane) {
            printf(" | State: Keep Lane | ");

            Vehicle *closest_vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
            bool has_vehicle_ahead = closest_vehicle_ahead != NULL;
            if (has_vehicle_ahead) {
                double check_car_s = closest_vehicle_ahead->s;
                double check_car_speed = closest_vehicle_ahead->speed;

                printf(" | Closest car speed: %f | ", check_car_speed);

                double distance = std::abs(check_car_s - this->cur_vehicle->s);

//                check_car_s += ((double) prev_size * .02 * check_car_speed);

                if (check_car_s > cur_vehicle->s && distance < this->min_safe_distance_threshold) {
                    if (check_car_speed <= speed_limit) {
                        // check if car ahead us behaves well. No reason to blindly follow a crazy driver
                        this->target_speed = check_car_speed;
                    }
                }
            } else {
                this->target_speed = this->speed_limit;
            }

            this->target_lane = this->cur_vehicle->lane;
        } else if (this->state == State::PrepareLaneChangeRight || this->state == State::PrepareLaneChangeLeft) {

            if (this->state == State::PrepareLaneChangeRight) {
                printf(" | State: Prepare Lane Change Right | ");
            } else {
                printf(" | State: Prepare Lane Change Left | ");
            }

            Vehicle *closest_vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
            bool has_vehicle_ahead = closest_vehicle_ahead != NULL;
            if (has_vehicle_ahead) {
                double closest_distance = std::abs(closest_vehicle_ahead->s - this->cur_vehicle->s);

                if (closest_distance <= this->min_safe_distance_threshold) {
                    this->target_speed = closest_vehicle_ahead->speed;
                } else {
                    this->target_speed = this->speed_limit;
                }
            } else {
                this->target_speed = this->speed_limit;
            }

            this->target_lane = this->cur_vehicle->lane;

        } else if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {

            if (this->state == State::LaneChangeRight) {
                printf(" | State: Lane Change Right | ");

                this->target_lane = this->cur_vehicle->lane + 1;
            } else {
                printf(" | State: Lane Change Left | ");

                this->target_lane = this->cur_vehicle->lane - 1;
            }

        }

        printf(" | Target speed: %f | ", this->target_speed);
        printf(" | Target lane: %d | ", this->target_lane);

        if (this->ref_velocity < this->target_speed) {
            printf(" | Speed policy: increase speed | ");
            this->ref_velocity = std::min(this->ref_velocity + this->velocity_change, this->target_speed);
        } else if (this->ref_velocity == this->target_speed) {
            printf(" | Speed policy: keep max speed | ");
        } else {
            printf(" | Speed policy: decrease speed | ");
            this->ref_velocity -= this->velocity_change;
        }
    }

    void Behavior::evaluate_keep_lane_trajectory(){
        Vehicle *vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
        bool has_vehicle_ahead = vehicle_ahead != NULL;

        if (has_vehicle_ahead) {
            double closest_distance = std::abs(vehicle_ahead->s - this->cur_vehicle->s);

            if (closest_distance > this->min_safe_distance_threshold && this->state == State::KeepLane) {
                return;
            }

            // TODO compute speed only ahead
            // TODO check collisions code

            vector<double> average_lane_speeds = road->get_average_lane_speeds();
            vector<double> closest_vehicles_in_lanes_speeds =
                    road->get_speed_of_closest_vehicles_for(this->cur_vehicle);

            int cur_lane = this->cur_vehicle->lane;

            double closest_speed_in_lane = closest_vehicles_in_lanes_speeds[cur_lane];
            double best_closest_speed = closest_speed_in_lane;

            for (int lane = 0; lane < closest_vehicles_in_lanes_speeds.size(); lane++) {
                printf(" | Closest vehicle speed %f in lane %d| ", closest_vehicles_in_lanes_speeds[lane], lane);

                if (closest_vehicles_in_lanes_speeds[lane] > best_closest_speed) {
                    best_closest_speed = closest_vehicles_in_lanes_speeds[lane];
                }
            }

            if (closest_speed_in_lane >= best_closest_speed) {
                // cur lane is better
                this->state = State::KeepLane;
                return;
            }

            int best_lane = -1;
            int best_average_lane_speed = -1;

            for (int possible_lane = 0; possible_lane < average_lane_speeds.size(); possible_lane++) {
                bool is_viable_lane = possible_lane != cur_lane && std::abs(cur_lane - possible_lane) <= 1;

                if (!is_viable_lane) {
                    continue;
                }

                printf(" | Average lane speed %f in lane %d| ", average_lane_speeds[possible_lane], possible_lane);

                if (best_average_lane_speed < 0 || average_lane_speeds[possible_lane] > best_average_lane_speed) {
                    best_average_lane_speed = average_lane_speeds[possible_lane];
                    best_lane = possible_lane;
                }
            }

            if (closest_speed_in_lane >= best_average_lane_speed) {
                // cur lane is better in the near future
                this->state = State::KeepLane;
                return;
            }

            if ((best_lane - cur_lane) > 0) {
                this->state = State::PrepareLaneChangeRight;
            } else if ((best_lane - cur_lane) < 0) {
                this->state = State::PrepareLaneChangeLeft;
            }
        }
    }

    double Behavior::get_ref_velocity() {
        return this->ref_velocity;
    }

    int Behavior::get_target_lane() {
        return this->target_lane;
    }

}
