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

// Initialise Global Values
void DynamicObjectDetection::init(json& sensor_fusion_data)
{
    DETECTIONS = sensor_fusion_data;
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
ForwardVehicle DynamicObjectDetection::CheckLane(int& lane, json& previous_path, double& car_s, double& end_traj_s)
{
    // Set ego position to end of previous trajectory of there is one
    if (previous_path.size() > 0)
    {
        car_s = end_traj_s;
    }

    // Create an object for the forward vehicle
    ForwardVehicle fv;

    // Initialise predicted s to arbitrary value
    fv.predicted_s = std::numeric_limits<double>::max();
    fv.rear_s = std::numeric_limits<double>::lowest();
    fv.too_close_front = false;
    fv.too_close_rear = false;

    // Iterate over sensor fusion data and perform checks relevant to center lane.
    for (int i = 0; i < DETECTIONS.size(); i++)
    {
        float d = DETECTIONS[i][6];
        // Determine if object in specified lane assuming lane width of 4m.
        if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
        {
        // Calculate Distance and speed to FV
        std::vector<double> vel_pred_s =
            DistanceSpeedCheck(DETECTIONS[i][3], DETECTIONS[i][4], previous_path.size());

            fv.lane_number = lane;
            fv.predicted_s = DETECTIONS[i][5];

            fv.velocity = vel_pred_s[0];
            fv.predicted_s += vel_pred_s[1];

            // Check if in front of ego
            if ((fv.predicted_s > car_s) && ((fv.predicted_s - car_s) < FV_MAX_HEADWAY))
            {
                fv.lane_free = false;
                fv.too_close_front = true;
            }
            else
            {
                fv.lane_free = true;
                fv.too_close_front = false;
            }

            return fv;
        }
    }
}
