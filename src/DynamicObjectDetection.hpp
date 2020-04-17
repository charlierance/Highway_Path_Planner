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
     * @brief Calculate the velocity and the predicted s coordinate in the future for the vehicle.
     * @param vx: Longitudinal velocity in m/s of the object.
     * @param vy: Lateral velocity in m/s of the object.
     * @param prev_size: The size of the previous trajectory.
     * @return A vector containing the magnitude of the velocity vector and the prediction of where that object will be
     * as a vector structured: [velocity, predicted_s].
     */
    std::vector<double> DistanceSpeedCheck(double vx, double vy, double prev_size);

    /**
     * @brief Return a ForwardVehicle object determining the state of the specified lane based on the dynamic objects.
     * @param sensor_fusion_data: The raw ouput of sensor fusion.
     * @param lane: Ego Vehicles Lane
     * @param previous_path: The ouput of the previous trajectory.
     * @param car_s: The cars s location in Frenet Coordinates.
     * @param end_traj_s: The end of the previous trajectory Frenet s coordinate.
     * @return A ForwardVehicle object describing the state of the dynmaic objects in the specified lane.
     */
    ForwardVehicle CheckLane(json& sensor_fusion_data, int& lane, json& previous_path, double& car_s,
                             double& end_traj_s);

    double FV_MAX_HEADWAY = 30.0;
    double MAX_REAR_HEADWAY = -40.0;
};

#endif  // PATH_PLANNING_DYNAMICOBJECTDETECTION_HPP
