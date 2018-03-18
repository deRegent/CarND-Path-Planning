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

        this->previous_path_x = previous_path_x;
        this->previous_path_y = previous_path_y;
        this->map_waypoints_x = map_waypoints_x;
        this->map_waypoints_y = map_waypoints_y;
        this->map_waypoints_s = map_waypoints_s;

        this->updateState();
        this->updateParams();
    }

    void Behavior::updateState() {

        printf("\r\n");
        printf("%s", this->has_collision_on_lane_change(0) ? "| X |" : "|  |");
        printf("%s", this->has_collision_on_lane_change(1) ? "| X |" : "|  |");
        printf("%s", this->has_collision_on_lane_change(2) ? "| X |" : "|  |");

        if (this->state == State::KeepLane) {

            this->evaluate_keep_lane_trajectory();

        } else if (this->state == State::PrepareLaneChangeRight || this->state == State::PrepareLaneChangeLeft) {

            this->evaluate_keep_lane_trajectory();

            int trajectory_lane;

            if (this->state == State::PrepareLaneChangeRight) {
                trajectory_lane = this->cur_vehicle->lane + 1;
            } else if (this->state == State::PrepareLaneChangeLeft) {
                trajectory_lane = this->cur_vehicle->lane - 1;
            } else {
                return;
            }

            bool has_collisions = this->has_collision_on_lane_change(trajectory_lane);

            printf("\r\n");
            printf("trajectory without collisions", has_collisions ? "| X |" : "|  |");

            if (!has_collisions) {
                if (this->state == State::PrepareLaneChangeRight) {
                    printf("\r\n");
                    printf("| State: Ready to change lane Right! |");
                    this->state = State::LaneChangeRight;
                    this->target_lane = this->cur_vehicle->lane + 1;
                } else {
                    printf("\r\n");
                    printf("| State: Ready to change lane Left! |");
                    this->state = State::LaneChangeLeft;
                    this->target_lane = this->cur_vehicle->lane - 1;
                }
            }
        } else if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {
            if (this->cur_vehicle->lane == this->target_lane) {
                this->state = State::KeepLane;
            }
        }
    }

    void Behavior::updateParams() {

        if (this->state == State::KeepLane) {
            printf("\r\n");
            printf("|State: Keep Lane|");

            this->follow_closest_vehicle();

        } else if (this->state == State::PrepareLaneChangeRight || this->state == State::PrepareLaneChangeLeft) {

            printf("\r\n");
            if (this->state == State::PrepareLaneChangeRight) {
                printf("|State: Prepare Lane Change Right|");
            } else {
                printf("|State: Prepare Lane Change Left|");
            }

            this->follow_closest_vehicle();

        } else if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {

            printf("\r\n");
            if (this->state == State::LaneChangeRight) {
                printf("|State: Lane Change Right|");
            } else {
                printf("|State: Lane Change Left|");
            }

        }

        printf("\r\n");
        printf("|Target speed: %f|", this->target_speed);
        printf("\r\n");
        printf("|Target lane: %d|", this->target_lane);

        printf("\r\n");
        if (this->ref_velocity < this->target_speed) {
            printf("|Speed policy: increase speed|");
            this->ref_velocity = std::min(this->ref_velocity + this->velocity_change, this->target_speed);
        } else if (this->ref_velocity == this->target_speed) {
            printf("|Speed policy: keep max speed|");
        } else {
            printf("|Speed policy: decrease speed|");
            this->ref_velocity -= this->velocity_change;
        }
    }

    void Behavior::evaluate_keep_lane_trajectory() {
        Vehicle *vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
        bool has_vehicle_ahead = vehicle_ahead != NULL;

        vector<double> average_lane_speeds = road->get_average_lane_speeds_ahead_of(this->cur_vehicle);
        vector<double> closest_vehicles_in_lanes_speeds =
                road->get_speed_of_closest_vehicles_for(this->cur_vehicle);

        int cur_lane = this->cur_vehicle->lane;

        double closest_speed_in_lane = closest_vehicles_in_lanes_speeds[cur_lane];
        double best_closest_speed = closest_speed_in_lane;

        int best_lane = -1;

        printf("\r\n");
        printf("|Closest speeds| ");

        for (int lane = 0; lane < closest_vehicles_in_lanes_speeds.size(); lane++) {

            bool viable = std::abs(lane - cur_lane) == 1;

            if (!viable){
                continue;
            }

            double vehicle_speed = closest_vehicles_in_lanes_speeds[lane];

            printf("| ");
            if (vehicle_speed != road->empty_lane_speed){
                printf("%f", vehicle_speed);
            } else {
                printf("INF");
                vehicle_speed = this->speed_limit;
            }
            printf(" |");

            if (vehicle_speed > best_closest_speed) {
                best_closest_speed = vehicle_speed;
                best_lane = lane;
            }
        }

        if (has_vehicle_ahead) {
            double closest_distance = std::abs(vehicle_ahead->s - this->cur_vehicle->s);

            if (closest_distance > this->min_safe_distance_threshold * 2 && this->state == State::KeepLane) {
                return;
            }

            if (closest_speed_in_lane >= best_closest_speed) {
                // cur lane is better
                this->state = State::KeepLane;

                printf("\r\n");
                printf(" |Current lane faster than best closest vehicles| ");

                return;
            }
//
            if ((best_lane - cur_lane) > 0) {
                this->state = State::PrepareLaneChangeRight;
            } else if ((best_lane - cur_lane) < 0) {
                this->state = State::PrepareLaneChangeLeft;
            }
        }
    }

    void Behavior::follow_closest_vehicle() {
        Vehicle *closest_vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
        bool has_vehicle_ahead = closest_vehicle_ahead != NULL;
        if (has_vehicle_ahead) {
            double check_car_s = closest_vehicle_ahead->s;
            double check_car_speed = closest_vehicle_ahead->speed;

            printf("\r\n");
            printf("|Closest car speed: %f|", check_car_speed);

//            check_car_s += ((double) prev_size * .02 * check_car_speed);

            double distance = std::abs(check_car_s - this->cur_vehicle->s);

            printf("\r\n");
            printf("|Distance to the closest car : %f|", distance);

            if (check_car_s > cur_vehicle->s && distance < this->min_safe_distance_threshold) {
                if (check_car_speed <= speed_limit) {
                    // check if car ahead us behaves well. No reason to blindly follow a crazy driver
                    this->target_speed = check_car_speed;
                }
            } else {
                this->target_speed = this->speed_limit;
            }
        } else {
            this->target_speed = this->speed_limit;
        }

        this->target_lane = this->cur_vehicle->lane;
    }

    bool Behavior::has_collision_on_lane_change(int trajectory_lane) {
        TrajectoryBuilder trajectoryBuilder;


        Trajectory trajectory = trajectoryBuilder.build_trajectory(this->cur_vehicle->x,
                                                                   this->cur_vehicle->y,
                                                                   this->cur_vehicle->s,
                                                                   this->cur_vehicle->yaw,
                                                                   trajectory_lane,
                                                                   this->ref_velocity,
                                                                   this->previous_path_x,
                                                                   this->previous_path_y,
                                                                   this->map_waypoints_x,
                                                                   this->map_waypoints_y,
                                                                   this->map_waypoints_s);

        bool has_collisions = road->has_collisions(trajectory, this->collision_threshold, trajectory_lane);

        return has_collisions;
    }

    double Behavior::get_ref_velocity() {
        return this->ref_velocity;
    }

    int Behavior::get_target_lane() {
        return this->target_lane;
    }

}
