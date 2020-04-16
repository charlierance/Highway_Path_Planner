//
// Created by charlie on 09/04/20.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATION_HPP
#define PATH_PLANNING_TRAJECTORYGENERATION_HPP

#include <vector>
#include "json.hpp"
#include "helpers.h"

using json = nlohmann::json;

class TrajectoryGeneration
{
  public:
    /**
     * @brief Default Constructor
     */
    TrajectoryGeneration();

    /**
     * @brief Default Destructor
     */
    virtual ~TrajectoryGeneration();

    /**
     * @brief Initialiser
     * @param ego_lane: The start lane for ego positioning.
     * @param max_vel: The maximum velocity ego cannot exceed.
     * @param previous_path: Previously generated trajectory vector
     * @param car_x: The current x coordinate of ego (cartesian).
     * @param car_y: The current y coordinate of ego (cartesian).
     * @param car_yaw: The current heading of ego in degrees.
     */
    void init(int& ego_lane, double& max_vel, json& previous_path, double& car_x,
              double& car_y, double& car_yaw);

    /**
     * @brief Find the last two path points from the previous trajectory,
     *        if non existent then use car position as estimation.
     * @param prev_x_points: The x coordinates previously passed to the planner (cartesian).
     * @param prev_y_points: The y coordinates previously passed to the planner (cartesian).
     * @param car_x: Ego vehicle current x coordinate.
     * @param car_y: Ego vehicle current y coordinate.
     * @param car_yaw: Ego vehicle current yaw in degrees.
     */
    void FindLastPoints(json& prev_x_points, json& prev_y_points, double& car_x,
                        double& car_y, double& car_yaw);

    /**
     * @brief Calculate a set number of points in the future trajectory and append them as x, y cartesian coordinates to
     *        ptsx and ptsy. It also calculates the scale factor of the distance based on the number of points required.
     * @param num_points: The number of points we wish to generate over the total distance.
     * @param car_s: Egos current s coordinate in Frenet coordiantes.
     * @param map_waypoints_s: The s coordinates from the given map waypoints.
     * @param map_waypoints_x: The x coordinates from the given map waypoints.
     * @param map_waypoints_y: The y coordinates from the given map waypoints.
     * @return An integer of the scale factor calculates based on the number of points vs the distance.
     */
    int FindNextPoints(int num_points, double& car_s, std::vector<double>& map_waypoints_s,
                       std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y);

    /**
     * @brief Takes the points that have been generated and converts them to the cars coordinate reference frame so
     *        trajectory xy starts at (0,0) and zero heading.
     */
    void ConvertToEgoCoordinates();

    /**
     * @brief Fits a spline for the trajectory. This method takes into account the previous path splines that have
     *        have already been consumed by the simulator and only returns the required amount of fitted splines.
     * @param scale_factor: The target distance per spline returned from FindNextPoints method.
     * @param prev_x_points: The vector of unused previous x trajectory points.
     * @param prev_y_points: The vector of unused previous y trajectory points.
     * @param next_x_vals: Vector of output x values.
     * @param next_y_vals: Vector of output y values.
     */
    void FitSplineTrajectory(int& scale_factor, json& prev_x_points, json& prev_y_points,
                             std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

  private:

    // Define const and parameters variables
    int EGO_LANE;
    double MAX_VEL;
    int MAX_TRAJECTORY_POINTS= 50;
    int MAX_TRAJECTORY_LENGTH = 90;

    // Previous trajectory size
    int prev_size;

    // Define the car current positioning
    double ref_x;
    double ref_y;
    double ref_yaw;

    // Define vector to store x and y points
    std::vector<double> ptsx;
    std::vector<double> ptsy;
};

#endif  // PATH_PLANNING_TRAJECTORYGENERATION_HPP
