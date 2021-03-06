#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "DynamicObjectDetection.hpp"
#include "TrajectoryGeneration.hpp"
#include "BehaviourStateMachine.hpp"
#include "diagnotics.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // Set variable params
    int EGOLANE = 1;
    double EGO_VEL = 0.0;

    h.onMessage([&EGOLANE, &EGO_VEL, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx,
                 &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */
                    //                    double SPEED_LIMIT = 49.5;
                    //                    double MAX_ACCELERATION = 0.3;
                    //
                    //                    // DoD
                    //                    DynamicObjectDetection DoD;
                    //                    DoD.init(sensor_fusion);
                    //                    auto forward_vehicle = DoD.CheckLane(EGOLANE, previous_path_x, car_s,
                    //                    end_path_s); std::cout << "Too Close?: " << forward_vehicle.too_close_front<<
                    //                    std::endl; std::cout << "Distance: " << forward_vehicle.predicted_s-car_s<<
                    //                    std::endl;
                    //
                    //                    // Slowdown Logic
                    //                    if (forward_vehicle.too_close_front){
                    //                        MAX_VEL -= MAX_ACCELERATION;
                    //                    }
                    //                    else if (MAX_VEL < SPEED_LIMIT) {
                    //                        MAX_VEL += MAX_ACCELERATION;
                    //                    }
                    //
                    //                    // Generate Trajectory
                    //                    TrajectoryGeneration tj;
                    //                    tj.init(EGOLANE, MAX_VEL, previous_path_x, car_x, car_y, car_yaw);
                    //                    tj.FindLastPoints(previous_path_x, previous_path_y, car_x, car_y, car_yaw);
                    //                    int scale = tj.FindNextPoints(3, car_s, map_waypoints_s, map_waypoints_x,
                    //                    map_waypoints_y); tj.ConvertToEgoCoordinates(); tj.FitSplineTrajectory(scale,
                    //                    previous_path_x, previous_path_y, next_x_vals, next_y_vals);

                    // Initialise Dynamic Object Detection
                    DynamicObjectDetection DoD;

                    // Define lane numbers, 0 = Furthest Left, 1 = Center, 2 = Furthest Right
                    std::vector<int> lanes = { 0, 1, 2 };

                    // Set a detection for each lane
                    auto lane_0_dod = DoD.CheckLane(sensor_fusion, lanes[0], previous_path_x, car_s, end_path_s);
                    auto lane_1_dod = DoD.CheckLane(sensor_fusion, lanes[1], previous_path_x, car_s, end_path_s);
                    auto lane_2_dod = DoD.CheckLane(sensor_fusion, lanes[2], previous_path_x, car_s, end_path_s);

                    // Initialise State Machine
                    BehaviourStateMachine state_machine;

                    // Calculate individual cost for each lane
                    double lane0_dist_cost = state_machine.FvDistanceCost(lane_0_dod, DoD.FV_MAX_HEADWAY, car_s);
                    double lane1_dist_cost = state_machine.FvDistanceCost(lane_1_dod, DoD.FV_MAX_HEADWAY, car_s);
                    double lane2_dist_cost = state_machine.FvDistanceCost(lane_2_dod, DoD.FV_MAX_HEADWAY, car_s);

                    double lane0_speed_cost = state_machine.LaneSpeedCost(lane_0_dod, state_machine.SPEED_LIMIT);
                    double lane1_speed_cost = state_machine.LaneSpeedCost(lane_1_dod, state_machine.SPEED_LIMIT);
                    double lane2_speed_cost = state_machine.LaneSpeedCost(lane_2_dod, state_machine.SPEED_LIMIT);

                    // Calculate total cost for each lane, adding slight bias for the center lane as this is the optimal
                    // lane.
                    double lane0_total_cost = ((lane0_dist_cost + lane0_speed_cost) / 3.0) + 0.2;
                    double lane1_total_cost = (lane1_dist_cost + lane1_speed_cost) / 3.0;
                    double lane2_total_cost = ((lane2_dist_cost + lane2_speed_cost) / 3.0) + 0.2;

                    // Decide on action using our state machine.
                    state_machine.FiniteStateMachine(EGO_VEL, EGOLANE, lane_0_dod, lane_1_dod, lane_2_dod,
                                                     lane0_total_cost, lane1_total_cost, lane2_total_cost);

                    // Generate our trajectory
                    TrajectoryGeneration tj;
                    tj.init(EGOLANE, EGO_VEL, previous_path_x, car_x, car_y, car_yaw);
                    tj.FindLastPoints(previous_path_x, previous_path_y, car_x, car_y, car_yaw);
                    int scale = tj.FindNextPoints(3, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    tj.ConvertToEgoCoordinates();
                    tj.FitSplineTrajectory(scale, previous_path_x, previous_path_y, next_x_vals, next_y_vals);

                    // Determine FV for diagnostics
                    double fv_distance, fv_speed;
                    bool fv_too_close;

                    if (EGOLANE == 0)
                    {
                        fv_distance = lane_0_dod.predicted_s - car_s;
                        fv_speed = lane_0_dod.velocity;
                        fv_too_close = lane_0_dod.too_close_front;
                    }
                    else if (EGOLANE == 1)
                    {
                        fv_distance = lane_1_dod.predicted_s - car_s;
                        fv_speed = lane_1_dod.velocity;
                        fv_too_close = lane_1_dod.too_close_front;
                    }
                    else if (EGOLANE == 2)
                    {
                        fv_distance = lane_2_dod.predicted_s - car_s;
                        fv_speed = lane_2_dod.velocity;
                        fv_too_close = lane_2_dod.too_close_front;
                    }

                    // Publish diagnostic output
                    PublishDiagnostics(EGOLANE, EGO_VEL, state_machine.STATE, fv_distance, fv_speed, fv_too_close,
                                       lane0_total_cost, lane1_total_cost, lane2_total_cost);

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    });    // end h.onMessage

    h.onConnection(
        [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}