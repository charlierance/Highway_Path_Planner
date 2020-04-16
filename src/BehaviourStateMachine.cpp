//
// Created by charlie on 09/04/20.
//

#include "BehaviourStateMachine.hpp"
#include "DynamicObjectDetection.hpp"
#include "TrajectoryGeneration.hpp"
#include "diagnotics.hpp"

using json = nlohmann::json;

// Default Constructor
BehaviourStateMachine::BehaviourStateMachine()
{
    DESIRED_EGO_LANE = 1;
}

// Default Destructor
BehaviourStateMachine::~BehaviourStateMachine()
{
}

// Calculate a cost value for the distance to the FV.
double BehaviourStateMachine::FvDistanceCost(ForwardVehicle& detection, double& max_headway, double& car_s)
{
    // First calculate a cost based on the distance to the FV
    double distance_cost;

    if ((detection.predicted_s - car_s) > max_headway)
    {
        distance_cost = 0.0;
    }
    else if ((detection.predicted_s - car_s) > 0.0001)
    {
        distance_cost = 1.0 - ((max_headway - (detection.predicted_s - car_s)) / max_headway);
    }
    else{
        distance_cost = 0.0;
    }

    return distance_cost;
}

double BehaviourStateMachine::LaneSpeedCost(ForwardVehicle& detection, double& optimum_speed)
{
    double speed_cost;

    double fv_mph = detection.velocity * 2.237;

    // FV greater than optimum speed incurs no cost
    if (fv_mph >= optimum_speed)
    {
        speed_cost = 0.0;
    }
    else if (fv_mph < optimum_speed)
    {
        speed_cost = (optimum_speed - fv_mph) / optimum_speed;
    }

    return speed_cost;
}

double BehaviourStateMachine::LaneChangeCost(int& lane, int& current_lane)
{
    double lane_cost;

    if (lane < 0.001 || lane > 2.001)
    {
        lane_cost = 1.0;
    }
    else if (current_lane == 0 && lane == 1)
    {
        lane_cost = 0.0;
    }
    else if ((current_lane == 1) && (lane == 0 || lane == 2))
    {
        lane_cost = 0.0;
    }
    else if (current_lane == 2 && lane == 1)
    {
        lane_cost = 0.0;
    }

    return lane_cost;
}

void BehaviourStateMachine::AdaptiveCruiseControl(ForwardVehicle& object_detection, double& ego_velocity)
{
    if ((object_detection.too_close_front) && ego_velocity != 0.0)
    {
        ego_velocity -= MAX_ACCELERATION;
    }
    else if (ego_velocity < SPEED_LIMIT)
    {
        ego_velocity += MAX_ACCELERATION;
    }
    else {
        ego_velocity = ego_velocity;
    }
}

void BehaviourStateMachine::LaneChange(int& desired_lane, int& ego_lane)
{
    // Calculate a binary cost relating to lane position
    double lane_cost = LaneChangeCost(desired_lane, ego_lane);

    // Only move if lane is plausible
    if (lane_cost < 1.0)
    {
        ego_lane = desired_lane;
    }
    else
    {
        ego_lane = ego_lane;
    }
}

void BehaviourStateMachine::FiniteStateMachine(json& sensor_fusion_data, double& EGO_SPEED, int& EGO_LANE,
                                               double& car_s, double& end_traj_s, json& previous_path_x,
                                               json& previous_path_y, double& car_d, double& car_x, double& car_y, double& car_yaw,
                                               vector<double>& map_waypoints_s, vector<double>& map_waypoints_x,
                                               vector<double>& map_waypoints_y, vector<double>& next_x_vals,
                                               vector<double>& next_y_vals)
{
    // Initialise Dynamic Object Detection
    DynamicObjectDetection DoD;
    DoD.init(sensor_fusion_data);

    // Define lane numbers, 0 = Furthest Left, 1 = Center, 2 = Furthest Right
    std::vector<int> lanes = { 0, 1, 2 };

    // Set a detection for each lane
    auto lane_0_dod = DoD.CheckLane(lanes[0], previous_path_x, car_s, end_traj_s);
    auto lane_1_dod = DoD.CheckLane(lanes[1], previous_path_x, car_s, end_traj_s);
    auto lane_2_dod = DoD.CheckLane(lanes[2], previous_path_x, car_s, end_traj_s);

    // Calculate individual cost for each lane
    double lane0_dist_cost = FvDistanceCost(lane_0_dod, DoD.FV_MAX_HEADWAY, car_s);
    double lane1_dist_cost = FvDistanceCost(lane_1_dod, DoD.FV_MAX_HEADWAY, car_s);
    double lane2_dist_cost = FvDistanceCost(lane_2_dod, DoD.FV_MAX_HEADWAY, car_s);

    double lane0_speed_cost = LaneSpeedCost(lane_0_dod, SPEED_LIMIT);
    double lane1_speed_cost = LaneSpeedCost(lane_1_dod, SPEED_LIMIT);
    double lane2_speed_cost = LaneSpeedCost(lane_2_dod, SPEED_LIMIT);

    // Calculate total cost for each lane, adding slight bias for the center lane as this is the optimal lane.
    double lane0_total_cost = ((lane0_dist_cost  + lane0_speed_cost) / 3.0) + 0.1;
    double lane1_total_cost = (lane1_dist_cost  + lane1_speed_cost) / 3.0;
    double lane2_total_cost = ((lane2_dist_cost  + lane2_speed_cost) / 3.0) + 0.1;

    string STATE;

    // State machine for lane 0
    if ((EGO_LANE == 0) && (lane0_total_cost < lane1_total_cost))
    {

        STATE = "ACC";
    }
    else if ((EGO_LANE == 0) && (lane0_total_cost > lane1_total_cost))
    {
        DESIRED_EGO_LANE = 1;

        if (lane_1_dod.lane_free){
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Right";
        }
        else{
            STATE = "Lane Change Right - BLOCKED";
            AdaptiveCruiseControl(lane_0_dod, EGO_SPEED);
        }
    }

    // State machine for lane 1
    if (EGO_LANE == 1 && (lane1_total_cost < lane0_total_cost) && (lane1_total_cost < lane2_total_cost))
    {
        AdaptiveCruiseControl(lane_1_dod, EGO_SPEED);
        STATE = "ACC";
    }
    else if ((EGO_LANE == 1) && (lane1_total_cost > lane0_total_cost) && (lane1_total_cost < lane2_total_cost))
    {
        DESIRED_EGO_LANE = 0;

        if (lane_0_dod.lane_free){
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Left";
        }
        else{
            STATE = "Lane Change Left - BLOCKED";
            AdaptiveCruiseControl(lane_1_dod, EGO_SPEED);
        }
    }
    else if ((EGO_LANE == 1) && (lane1_total_cost < lane0_total_cost) && (lane1_total_cost > lane2_total_cost))
    {
        DESIRED_EGO_LANE = 2;

        if (lane_2_dod.lane_free){
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Right";
        }
        else{
            STATE = "Lane Change Right - BLOCKED";
            AdaptiveCruiseControl(lane_1_dod, EGO_SPEED);
        }
    }

    // State machine for lane 2
    if ((EGO_LANE == 2) && (lane2_total_cost < lane1_total_cost))
    {
        AdaptiveCruiseControl(lane_2_dod, EGO_SPEED);
        STATE = "ACC";
    }
    else if ((EGO_LANE == 2) && (lane2_total_cost > lane1_total_cost))
    {
        DESIRED_EGO_LANE = 1;

        if (lane_1_dod.lane_free){
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Left";
        }
        else{
            STATE = "Lane Change Left - BLOCKED";
            AdaptiveCruiseControl(lane_2_dod, EGO_SPEED);
        }
    }

    // Generate our trajectory
    TrajectoryGeneration tj;
    tj.init(EGO_LANE, EGO_SPEED, previous_path_x, car_x, car_y, car_yaw);
    tj.FindLastPoints(previous_path_x, previous_path_y, car_x, car_y, car_yaw);
    int scale = tj.FindNextPoints(3, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    tj.ConvertToEgoCoordinates();
    tj.FitSplineTrajectory(scale, previous_path_x, previous_path_y, next_x_vals, next_y_vals);

    // Determine FV for diagnostics
    double fv_distance, fv_speed;
    bool fv_too_close;

    if (EGO_LANE == 0){
        fv_distance = lane_0_dod.predicted_s - car_s;
        fv_speed = lane_0_dod.velocity;
        fv_too_close = lane_0_dod.too_close_front;

        cout << "PREDICTED_S: " << lane_0_dod.predicted_s << endl;
        cout << "CAR_S: " << car_s << endl;
    }
    else if (EGO_LANE == 1){
        fv_distance = lane_1_dod.predicted_s - car_s;
        fv_speed = lane_1_dod.velocity;
        fv_too_close = lane_1_dod.too_close_front;

        cout << "PREDICTED_S: " << lane_1_dod.predicted_s << endl;
        cout << "CAR_S: " << car_s << endl;
    }
    else if (EGO_LANE == 2){
        fv_distance = lane_2_dod.predicted_s - car_s;
        fv_speed = lane_2_dod.velocity;
        fv_too_close = lane_2_dod.too_close_front;

        cout << "PREDICTED_S: " << lane_2_dod.predicted_s << endl;
        cout << "CAR_S: " << car_s << endl;
    }



    // Publish diagnostic output
    PublishDiagnostics(EGO_LANE, EGO_SPEED, STATE, fv_distance, fv_speed, fv_too_close, lane0_total_cost, lane1_total_cost, lane2_total_cost);
}