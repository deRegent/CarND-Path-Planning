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

        milliseconds cur_time = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
        );

        this->observation_time = cur_time;

        printf("\r\n");
        printf("Current lane: %d", this->cur_vehicle->lane);
        printf("%s", this->has_collision_on_lane_change(0) ? "| X |" : "|  |");
        printf("%s", this->has_collision_on_lane_change(1) ? "| X |" : "|  |");
        printf("%s", this->has_collision_on_lane_change(2) ? "| X |" : "|  |");

        this->update_road_observation();

        if (this->state == State::KeepLane) {

            double keep_lane_cost = this->evaluate_next_state(State::KeepLane);
            double lane_change_right_cost = this->evaluate_next_state(State::PrepareLaneChangeRight);
            double lane_change_left_cost = this->evaluate_next_state(State::PrepareLaneChangeLeft);

            printf("\r\n");
            printf("|KL: %f|PLCR: %f|PLCL: %f|", keep_lane_cost, lane_change_right_cost, lane_change_left_cost);

            vector<double> costs;
            costs.push_back(keep_lane_cost);
            costs.push_back(lane_change_right_cost);
            costs.push_back(lane_change_left_cost);

            double min = minAt(costs);

            if (lane_change_right_cost == min) {
                this->state = State::PrepareLaneChangeRight;
            } else if (lane_change_left_cost == min) {
                this->state = State::PrepareLaneChangeLeft;
            } else {
                this->state = State::KeepLane;
            }

        } else if (this->state == State::PrepareLaneChangeRight || this->state == State::PrepareLaneChangeLeft) {

            double keep_lane_cost = this->evaluate_next_state(State::KeepLane);
            double lane_change_right_cost = this->evaluate_next_state(State::PrepareLaneChangeRight);
            double lane_change_left_cost = this->evaluate_next_state(State::PrepareLaneChangeLeft);

            printf("\r\n");
            printf("|KL: %f|PLCR: %f|PLCL: %f|", keep_lane_cost, lane_change_right_cost, lane_change_left_cost);

            vector<double> costs;
            costs.push_back(keep_lane_cost);
            costs.push_back(lane_change_right_cost);
            costs.push_back(lane_change_left_cost);

            double min = minAt(costs);

            if (lane_change_right_cost == min) {
                this->state = State::PrepareLaneChangeRight;
            } else if (lane_change_left_cost == min) {
                this->state = State::PrepareLaneChangeLeft;
            } else {
                this->state = State::KeepLane;
                return;
            }

            double change_lane_cost =
                    this->state == State::PrepareLaneChangeRight ? this->evaluate_next_state(State::LaneChangeRight)
                                                                 : this->evaluate_next_state(State::LaneChangeLeft);
            bool change_lane = change_lane_cost < lane_change_right_cost;

            printf("\r\n");
            printf("|CL: %f|PLCR: %f|PLCL: %f|", change_lane_cost, lane_change_right_cost, lane_change_left_cost);

            if (change_lane) {
                if (this->state == State::PrepareLaneChangeRight) {
                    this->state = State::LaneChangeRight;
                    this->target_lane = this->cur_vehicle->lane + 1;
                    this->target_speed = this->ref_velocity;
                } else {
                    this->state = State::LaneChangeLeft;
                    this->target_lane = this->cur_vehicle->lane - 1;
                    this->target_speed = this->ref_velocity;
                }
            }
        } else if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {
            double keep_lane_cost = this->evaluate_next_state(State::KeepLane);
            double change_lane_cost =
                    this->state == State::LaneChangeRight ? this->evaluate_next_state(State::LaneChangeRight)
                                                          : this->evaluate_next_state(State::LaneChangeLeft);

            printf("\r\n");
            printf("|KL: %f|CL: %f|", keep_lane_cost, change_lane_cost);

            if (keep_lane_cost < change_lane_cost) {
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

    void Behavior::follow_closest_vehicle() {
        Vehicle *closest_vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
        bool has_vehicle_ahead = closest_vehicle_ahead != NULL;
        if (has_vehicle_ahead) {
            double check_car_s = closest_vehicle_ahead->s;
            double check_car_speed = closest_vehicle_ahead->speed;

            printf("\r\n");
            printf("|Closest car speed: %f|", check_car_speed);

            check_car_s += this->previous_path_x.size() * 0.02 * closest_vehicle_ahead->speed;
            double distance = std::abs(check_car_s - this->cur_vehicle->s);

            printf("\r\n");
            printf("|Distance to the closest car : %f|", distance);

            if (check_car_s > cur_vehicle->s && distance < this->min_safe_distance_threshold) {
                // check if car ahead us behaves well. No reason to blindly follow a crazy driver
                this->target_speed = std::min(check_car_speed, this->speed_limit);
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

    void Behavior::update_road_observation() {
        Vehicle *closest_vehicle_ahead = road->get_closest_vehicle_ahead_of(this->cur_vehicle);
        bool has_vehicle_ahead = closest_vehicle_ahead != NULL;
        if (has_vehicle_ahead) {
            double check_car_s = closest_vehicle_ahead->s;
            double check_car_speed = closest_vehicle_ahead->speed;

            printf("\r\n");
            printf("|Closest car speed: %f|", check_car_speed);

            check_car_s += this->previous_path_x.size() * 0.02 * closest_vehicle_ahead->speed;

            double distance = std::abs(check_car_s - this->cur_vehicle->s);

            printf("\r\n");
            printf("|Distance to the closest car : %f|", distance);

            if (distance < this->block_distance_threshold) {
                this->blocked_in_lane = true;
                printf("\r\n");
                printf("|Blocked|");
                return;
            }
        }
        this->blocked_in_lane = false;
    }

    double Behavior::evaluate_lane_speed(int lane) {
        vector<double> speeds = road->get_speed_of_closest_vehicles_for(this->cur_vehicle);
        for (int i = 0; i < speeds.size(); i++) {
            if (speeds[i] > this->speed_limit) {
                speeds[i] = this->speed_limit;
            }
        }

        double max = maxAt(speeds);

        return (1 - (speeds[lane] / max)) + 0.1;
    }

    double Behavior::evaluate_next_state(State state) {

        if (state == State::KeepLane) {

            if (this->state == State::LaneChangeRight || this->state == State::LaneChangeLeft) {
                if (this->cur_vehicle->lane == this->target_lane) {
                    if (this->last_lane_change_millisec == 0) {
                        this->last_lane_change_millisec = this->observation_time.count();
                    }
                    auto time_diff = this->observation_time.count() - this->last_lane_change_millisec;
                    if (time_diff > this->lane_transition_millisec) {
                        this->last_lane_change_millisec = 0;
                        return 0;
                    }
                }
                return 1;
            }

            if (this->blocked_in_lane) {
                return this->evaluate_lane_speed(this->cur_vehicle->lane);
            } else {
                return 0;
            }

        } else if (state == State::PrepareLaneChangeLeft) {

            if (this->state == State::LaneChangeLeft || this->state == State::LaneChangeRight) {
                return 1;
            }

            int next_lane = this->cur_vehicle->lane - 1;
            if (next_lane < 0) {
                return 1;
            }
            return this->evaluate_lane_speed(next_lane);

        } else if (state == State::PrepareLaneChangeRight) {

            if (this->state == State::LaneChangeLeft || this->state == State::LaneChangeRight) {
                return 1;
            }

            int next_lane = this->cur_vehicle->lane + 1;
            if (next_lane >= road->lanes) {
                return 1;
            }
            return this->evaluate_lane_speed(next_lane);

        } else if (state == State::LaneChangeRight) {

            if (this->state == State::PrepareLaneChangeRight) {

                int next_lane = this->cur_vehicle->lane + 1;

                if (this->has_collision_on_lane_change(next_lane)) {
                    return 1;
                }

                return 0;
            } else if (this->state == State::LaneChangeRight) {
                return 0.5;
            } else {
                return 1;
            }

        } else if (state == State::LaneChangeLeft) {
            if (this->state == State::PrepareLaneChangeLeft) {

                int next_lane = this->cur_vehicle->lane - 1;

                if (this->has_collision_on_lane_change(next_lane)) {
                    return 1;
                }

                return 0;
            } else if (this->state == State::LaneChangeLeft) {
                return 0.5;
            } else {
                return 1;
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
