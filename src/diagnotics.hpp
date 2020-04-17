//
// Created by charlie on 16/04/20.
//

#ifndef PATH_PLANNING_DIAGNOTICS_HPP
#define PATH_PLANNING_DIAGNOTICS_HPP

#include <iostream>

using namespace std;

// Publish a diagnostic output to STDOUT
inline void PublishDiagnostics(int& ego_lane, double& ego_velocity, string& state, double& Forward_Vehicle_Distance,
                               double& Forward_Vehicle_Speed, bool& fv_too_close, double& lane_0_cost,
                               double& lane_1_cost, double& lane_2_cost)
{
    cout << "************************************" << endl;
    cout << "Ego Lane                  : " << ego_lane << endl;
    cout << "Ego Speed                 : " << ego_velocity << endl;
    cout << "Planner State             : " << state << endl;
    cout << "Forward Vehicle Too Close : " << fv_too_close << endl;
    cout << "Forward Vehicle Distance  : " << Forward_Vehicle_Distance << endl;
    cout << "Forward Vehicle Speed     : " << Forward_Vehicle_Speed << endl;
    cout << "Lane 0 Total Cost         : " << lane_0_cost << endl;
    cout << "Lane 1 Total Cost         : " << lane_1_cost << endl;
    cout << "Lane 2 Total Cost         : " << lane_2_cost << endl;
    cout << "************************************" << endl;
}

#endif  // PATH_PLANNING_DIAGNOTICS_HPP
