//
// Created by Дима on 13.03.2018.
//

#include "vehicle.h"

namespace car_nd_path_planning {
    using namespace std;

    Vehicle::Vehicle(double x, double y, double yaw, double s, double d) {
        double vx = speed*cos(yaw);
        double vy = speed*sin(yaw);

        this->init(this->default_car_id, x, y, vx, vy, s, d);
    }

    Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d) {
        this->init(id, x, y, vx, vy, s, d);
    }

    void Vehicle::init(int id, double x, double y, double vx, double vy, double s, double d){
        this->id = id;
        this->x = x;
        this->y = y;
        this->vx = vx;
        this->vy = vy;
        this->s = s;
        this->d = d;

        this->speed = sqrt(vx * vx + vy * vy);;
        this->lane = floor(d / 4.0);
        this->acceleration_x = 0;
        this->acceleration_y = 0;

        milliseconds cur_time = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
        );

        this->last_update_time = cur_time;
    }

    void Vehicle::update(double x, double y, double yaw, double s, double d) {
        double vx = speed*cos(yaw);
        double vy = speed*sin(yaw);

       this->update(x, y, vx, vy, s, d);
    }

    void Vehicle::update(double x, double y, double vx, double vy, double s, double d) {
        milliseconds cur_time = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
        );

        auto delta_t = (double) (cur_time - this->last_update_time).count() / 1000;

        this->last_update_time = cur_time;

        this->acceleration_x = (vx - this->vx) / delta_t;
        this->acceleration_y = (vy - this->vy) / delta_t;

        this->x = x;
        this->y = y;
        this->vx = vx;
        this->vy = vy;
        this->s = s;
        this->d = d;

        this->speed = sqrt(vx * vx + vy * vy);;
        this->lane = floor(d / 4.0);
    }

    Trajectory Vehicle::predict_trajectory(int horizon, double delta_t) {
        vector<double> next_x_vals;
        vector<double> next_y_vals;

        double x = this->x;
        double vx = this->vx;
        double y = this->y;
        double vy = this->vy;

        for (int i = 0; i < horizon; i++) {
            x = x + vx * delta_t + this->acceleration_x * delta_t * delta_t / 2;
            vx = vx + this->acceleration_x * delta_t;

            y = y + vy * delta_t + this->acceleration_y * delta_t * delta_t / 2;
            vy = vy + this->acceleration_y * delta_t;

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
        }

        Trajectory trajectory(next_x_vals, next_y_vals);

        return trajectory;
    }

}