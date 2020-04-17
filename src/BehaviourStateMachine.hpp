//
// Created by charlie on 09/04/20.
//

#ifndef PATH_PLANNING_BEHAVIOURSTATEMACHINE_HPP
#define PATH_PLANNING_BEHAVIOURSTATEMACHINE_HPP

#include "DynamicObjectDetection.hpp"

using json = nlohmann::json;

class BehaviourStateMachine
{
  public:
    /**
     * @brief Default Constructor
     */
    BehaviourStateMachine();

    /**
     * @brief Default Destructor
     */
    virtual ~BehaviourStateMachine();

    /**
     * @brief Calculate a cost for the lane based on the vehicle distance and if it is under the minimum headway.
     * @param detection: The Dynamic Object Detections for that lane.
     * @param max_headway: The defined parameter for the minimum headway distance.
     * @param car_s: Egos s position in Frenet Coordinates.
     * @return The cost value as a double.
     */
    double FvDistanceCost(ForwardVehicle& detection, double& max_headway, double& car_s);

    /**
     * @brief Determines a cost per lane dependant on the speed of the dynamic objects in that lane.
     * @param detection: The Dynamic Object Detections for that lane.
     * @param optimum_speed: The optimum speed that ego should be traveling at.
     * @return The cost value as a double.
     */
    double LaneSpeedCost(ForwardVehicle& detection, double& optimum_speed);

    /**
     * @brief Determine a cost for a lane change, this is a binary function intended to penalise illegal maneuvers.
     * @param lane: The lane we intend ego to change to.
     * @param current_lane: The current lane ego is in.
     * @return The cost value as a double.
     */
    double LaneChangeCost(int& lane, int& current_lane);

    /**
     * @brief Modulate egos speed dependant on the distance to the forward vehicle.
     * @param object_detection: The Dynamic Object Detections for Ego's Lane.
     * @param ego_velocity: Ego Vehicles Current Velocity.
     */
    void AdaptiveCruiseControl(ForwardVehicle& object_detection, double& ego_velocity);

    /**
     * @brief Perform a lane change to the desired lane.
     * @param desired_lane: The intended lane for ego to move too.
     * @param ego_lane: Ego vehicles current lane.
     */
    void LaneChange(int& desired_lane, int& ego_lane);

    /**
     * @brief This is the state machine that determines the action that ego should perform.
     * @param EGO_SPEED: Speed of ego vehicle
     * @param EGO_LANE: Ego current lane
     * @param lane_0_dod: Output of DoD for lane 0.
     * @param lane_1_dod: Output of DoD for lane 1.
     * @param lane_2_dod: Output of DoD for lane 2.
     * @param lane0_total_cost: The normalised and weighted sum of all the cost functions for lane 0.
     * @param lane1_total_cost: The normalised and weighted sum of all the cost functions for lane 1.
     * @param lane2_total_cost: The normalised and weighted sum of all the cost functions for lane 2.
     */
    void FiniteStateMachine(double& EGO_SPEED, int& EGO_LANE, ForwardVehicle lane_0_dod, ForwardVehicle lane_1_dod,
                            ForwardVehicle lane_2_dod, double lane0_total_cost, double lane1_total_cost,
                            double lane2_total_cost);

    std::string STATE;
    double MAX_ACCELERATION = 0.3;
    double SPEED_LIMIT = 49.5;
    int DESIRED_EGO_LANE;
};

#endif  // PATH_PLANNING_BEHAVIOURSTATEMACHINE_HPP
