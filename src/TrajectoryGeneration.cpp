//
// Created by charlie on 09/04/20.
//

#include <vector>
#include <iostream>
#include "TrajectoryGeneration.hpp"
#include "spline.hpp"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

// Constructor
TrajectoryGeneration::TrajectoryGeneration()
{
}

// Destructor
TrajectoryGeneration::~TrajectoryGeneration()
{
}

// Initialise Variables
void TrajectoryGeneration::init(int& ego_lane, double& max_vel, json& previous_path,
                                double& car_x, double& car_y, double& car_yaw)
{
    // Define const and parameters variables
    EGO_LANE = ego_lane;
    MAX_VEL = max_vel;

    // Previous trajectory size
    prev_size = previous_path.size();

    // Define the car current positioning
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);
}

// Find points from previous trajectory
void TrajectoryGeneration::FindLastPoints(json& prev_x_points, json& prev_y_points, double& car_x,
                                          double& car_y, double& car_yaw)
{
    // Make sure there are two previous points to use if not use ego pose
    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }

    else
    {
        ref_x = prev_x_points[prev_size - 1];
        ref_y = prev_y_points[prev_size - 1];

        double ref_x_prev = prev_x_points[prev_size - 2];
        double ref_y_prev = prev_y_points[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Add the two endpoints from the previous path
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
}

// Find next waypoints
int TrajectoryGeneration::FindNextPoints(int num_points, double& car_s, vector<double>& map_waypoints_s,
                                         vector<double>& map_waypoints_x, vector<double>& map_waypoints_y)
{
    // Verify num_points is a factor of dist
    if (MAX_TRAJECTORY_LENGTH % num_points != 0)
    {
        cout << "ERROR: num_points " << num_points << " is not a factor of dist " << MAX_TRAJECTORY_LENGTH
             << " when finding future points!" << endl;
        exit(EXIT_FAILURE);
    }

    // Distance calculation
    int scale_factor = MAX_TRAJECTORY_LENGTH / num_points;

    while (scale_factor <= MAX_TRAJECTORY_LENGTH)
    {
        // Calculate next coord in xy
        vector<double> next_waypoint =
            getXY(car_s + scale_factor, (2 + 4 * EGO_LANE), map_waypoints_s, map_waypoints_x, map_waypoints_y);

        // Update vectors
        ptsx.push_back(next_waypoint[0]);
        ptsy.push_back(next_waypoint[1]);

        scale_factor += scale_factor;
    }

    // Return scale factor for later use
    return MAX_TRAJECTORY_LENGTH / num_points;
}

// Zero trajectory coordinates and heading
void TrajectoryGeneration::ConvertToEgoCoordinates()
{
    for (int i = 0; i < ptsx.size(); i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
}

// Spline fitting algorithm
void TrajectoryGeneration::FitSplineTrajectory(int& scale_factor, json& prev_x_points, json& prev_y_points, vector<double>& next_x_vals,
                                               vector<double>& next_y_vals)
{
    // Initialise our spline
    tk::spline s;
    s.set_points(ptsx, ptsy);


    // Start by using our previous path points that have not been consumed
    for (int i = 0; i < prev_size; i++)
    {
        next_x_vals.push_back(prev_x_points[i]);
        next_y_vals.push_back(prev_y_points[i]);
    }

    // Calculate the split of the points on the spline
    double target_x = scale_factor;
    double target_y = s(target_x);
    double target_distance = sqrt((target_x*target_x)+(target_y*target_y));

    double x_addon = 0.0;

    // Now split the spline based on our overall trajectory length and those points that have been consumed.
    for (int j = 1; j < MAX_TRAJECTORY_POINTS - prev_size; j++)
    {
        // Define an interval to reach a distance based on a velocity, not here we convert MPH to m/s
        double N = (target_distance/(0.02*MAX_VEL/2.24));

        // Define our x based upon the interval and use our spline fitting to find the y
        double x_point = x_addon + target_x / N;
        double y_point = s(x_point);

        x_addon = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Convert coordinates back to the map reference frame
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}