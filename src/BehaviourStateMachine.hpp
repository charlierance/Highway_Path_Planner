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
     * @brief Initialisation Function
     * @param Ego Vehicles Current Lane
     */
    void init(int& EGO_LANE);

    double FvDistanceCost(ForwardVehicle& detection, double& max_headway, double& car_s);

    double RearDistanceCost(ForwardVehicle& detection, double& max_rear_headway, double& car_s);

    double LaneSpeedCost(ForwardVehicle& detection, double& optimum_speed);

    double LaneChangeCost(int& lane, int& current_lane);

    void AdaptiveCruiseControl(ForwardVehicle& object_detection, double& ego_velocity);

    void LaneChange(int& desired_lane, int& ego_lane);

    void FiniteStateMachine(json& sensor_fusion_data, double& EGO_SPEED, int& EGO_LANE, double& car_s,
                            double& end_traj_s, json& previous_path_x, json& previous_path_y, double& car_d, double& car_x,
                            double& car_y, double& car_yaw, std::vector<double>& map_waypoints_s,
                            std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y,
                            std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

  private:
    double MAX_ACCELERATION = 0.3;
    double SPEED_LIMIT = 49.5;
    int DESIRED_EGO_LANE;
};

#endif  // PATH_PLANNING_BEHAVIOURSTATEMACHINE_HPP
