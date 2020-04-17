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
    else
    {
        distance_cost = 0.0;
    }

    return distance_cost;
}

// Determine lane to change to based on the speed vehicles are traveling.
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

// Determine cost for lane change and heavily penalise for illegal maneuver
double BehaviourStateMachine::LaneChangeCost(int& lane, int& current_lane)
{
    double lane_cost;

    if (lane < -0.001 || lane > 2.001)
    {
        lane_cost = 1.0;
    }
    else if ((current_lane == 0) && (lane == 1))
    {
        lane_cost = 0.0;
    }
    else if ((current_lane == 1) && (lane == 0 || lane == 2))
    {
        lane_cost = 0.0;
    }
    else if ((current_lane == 2) && (lane == 1))
    {
        lane_cost = 0.0;
    }
    else
    {
        lane_cost = 1.0;
    }

    return lane_cost;
}

// Acc Functionality
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
    else
    {
        ego_velocity = ego_velocity;
    }
}

// Lane change function based on cost
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

void BehaviourStateMachine::FiniteStateMachine(double& EGO_SPEED, int& EGO_LANE, ForwardVehicle lane_0_dod,
                                               ForwardVehicle lane_1_dod, ForwardVehicle lane_2_dod,
                                               double lane0_total_cost, double lane1_total_cost,
                                               double lane2_total_cost)
{
    // State machine for lane 0
    if ((EGO_LANE == 0) && (lane0_total_cost < lane1_total_cost))
    {
        AdaptiveCruiseControl(lane_0_dod, EGO_SPEED);
        STATE = "ACC";
    }
    else if ((EGO_LANE == 0) && (lane0_total_cost > lane1_total_cost))
    {
        DESIRED_EGO_LANE = 1;

        if (lane_1_dod.lane_free)
        {
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Right";
        }
        else
        {
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

        if (lane_0_dod.lane_free)
        {
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Left";
        }
        else
        {
            STATE = "Lane Change Left - BLOCKED";
            AdaptiveCruiseControl(lane_1_dod, EGO_SPEED);
        }
    }
    else if ((EGO_LANE == 1) && (lane1_total_cost < lane0_total_cost) && (lane1_total_cost > lane2_total_cost))
    {
        DESIRED_EGO_LANE = 2;

        if (lane_2_dod.lane_free)
        {
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Right";
        }
        else
        {
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

        if (lane_1_dod.lane_free)
        {
            LaneChange(DESIRED_EGO_LANE, EGO_LANE);
            STATE = "Lane Change Left";
        }
        else
        {
            STATE = "Lane Change Left - BLOCKED";
            AdaptiveCruiseControl(lane_2_dod, EGO_SPEED);
        }
    }
}