//
// Created by charlie on 09/04/20.
//

#include "DynamicObjectDetection.hpp"
#include <iostream>

using json = nlohmann::json;

// Constructor
DynamicObjectDetection::DynamicObjectDetection()
{
}

// Destructor
DynamicObjectDetection::~DynamicObjectDetection()
{
}

// Return Lane number 0 = Left, 1 = Center, 2 = Right
double GetCarLane(double car_d)
{
    double lane = 1;  // default CENTER lane
    if (car_d > 0 && car_d < 4)
    {
        lane = 0;  // LEFT
    }
    else if (car_d >= 4 && car_d <= 8)
    {
        lane = 1;  // CENTER
    }
    else if (car_d > 8 && car_d <= 12)
    {
        lane = 2;  // RIGHT
    }
    return lane;
}

// Calculation of velocity and distance
std::vector<double> DynamicObjectDetection::DistanceSpeedCheck(double vx, double vy, double prev_size)
{
    double speed_magnitude = sqrt(vx * vx + vy * vy);
    double predicted_s = prev_size * 0.02 * speed_magnitude;

    std::vector<double> vel_pred_s = { speed_magnitude, predicted_s };

    return vel_pred_s;
}

// Determine FV in Ego Lane
ForwardVehicle DynamicObjectDetection::CheckLane(json& sensor_fusion_data, int& lane, json& previous_path,
                                                 double& car_s, double& end_traj_s)
{
    // Set ego position to end of previous trajectory of there is one
    if (previous_path.size() > 0)
    {
        car_s = end_traj_s;
    }

    double closest_front = std::numeric_limits<double>::max();
    double closest_rear = -std::numeric_limits<double>::max();

    double vehicle_speed;

    // Iterate over sensor fusion data and perform checks relevant to center lane.
    for (int i = 0; i < sensor_fusion_data.size(); i++)
    {
        float d = sensor_fusion_data[i][6];
        double vehicle_lane = GetCarLane(d);

        if (lane == vehicle_lane)
        {
            // Calculate Distance and speed to FV
            std::vector<double> vel_pred_s =
                DistanceSpeedCheck(sensor_fusion_data[i][3], sensor_fusion_data[i][4], previous_path.size());

            vehicle_speed = vel_pred_s[0];
            double predicted_s = (double)sensor_fusion_data[i][5] + vel_pred_s[1];

            if ((predicted_s > 0.0001) && (predicted_s > car_s))
            {
                closest_front = std::min(predicted_s, closest_front);
            }
            else if ((predicted_s <= 0.0001) && (predicted_s < car_s))
            {
                closest_rear = std::max(predicted_s, closest_rear);
            }
        }
    }

    ForwardVehicle fv;

    if ((closest_front > (car_s + FV_MAX_HEADWAY)) && (closest_rear < (car_s - MAX_REAR_HEADWAY)))
    {
        fv.lane_free = true;
    }
    else
    {
        fv.lane_free = false;
    }

    if (closest_front < (car_s + FV_MAX_HEADWAY))
    {
        fv.too_close_front = true;
    }
    else
    {
        fv.too_close_front = false;
    }

    if (closest_rear > (car_s - MAX_REAR_HEADWAY))
    {
        fv.too_close_rear = true;
    }
    else
    {
        fv.too_close_rear = false;
    }

    fv.lane_number = lane;
    fv.velocity = vehicle_speed;
    fv.predicted_s = closest_front;
    fv.rear_s = closest_rear;

    return fv;
}
