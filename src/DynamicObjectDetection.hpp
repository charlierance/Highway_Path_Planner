//
// Created by charlie on 09/04/20.
//

#ifndef PATH_PLANNING_DYNAMICOBJECTDETECTION_HPP
#define PATH_PLANNING_DYNAMICOBJECTDETECTION_HPP

#include <vector>
#include "json.hpp"

using json = nlohmann::json;

struct ForwardVehicle
{
    bool lane_free;        // True if lane is unoccupied else False
    bool too_close_front;  // True if Too Close to FV Else False
    bool too_close_rear;   // True if Too Close to Rear Vehicle else False
    int lane_number;       // 0 = Left Lane, 1 = Center Lane, 2 = Outer Lane
    double velocity;       // Magnitude of vx and vy
    double predicted_s;    // Predicted Frenet S Coord based on velocity and previous trajectory length
    double rear_s;         // Predicted Frenet S Coord of the closes vehicle behind egos current pose.
};

class DynamicObjectDetection
{
  public:
    /**
     * @brief Default Constructor
     */
    DynamicObjectDetection();

    /**
     * @brief Default Destructor
     */
    virtual ~DynamicObjectDetection();

    /**
     * @brief Initialising the shared value in the class.
     * @param sensor_fusion_data: The JSON data produced by the output of sensor fusion.
     */
    void init(json& sensor_fusion_data);

    /**
     * @brief Calculate the velocity and the predicted s coordinate in the future for the vehicle.
     * @param vx: Longitudinal velocity in m/s of the object.
     * @param vy: Lateral velocity in m/s of the object.
     * @param prev_size: The size of the previous trajectory.
     * @return A vector containing the magnitude of the velocity vector and the prediction of where that object will be
     * as a vector structured: [velocity, predicted_s].
     */
    std::vector<double> DistanceSpeedCheck(double vx, double vy, double prev_size);

    /**
     * @brief Used to determine if a FV is in center lane and if so return its characteristics.
     * @param lane: An integer value of the lane to check, 0 = Left Lane, 1 = Center Lane, 2 = Outer Lane
     * @param previous_path: A json vector of the previous trajectory calculated.
     * @param car_s: Egos Frenet s coordinate.
     * @param end_traj_s: The end of the previous trajectorys' s coordinate.
     * @return A ForwardVehicle object containing relevant information about objects in the current lane.
     */
    ForwardVehicle CheckLane(int& lane, json& previous_path, double& car_s, double& end_traj_s);

    double FV_MAX_HEADWAY = 30.0;
    double FV_MAX_REAR_HEADWAY = 10.0;
    json DETECTIONS;
};

#endif  // PATH_PLANNING_DYNAMICOBJECTDETECTION_HPP
