#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "car.h"
#include "vehicle_controller.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void ReadWaypoints(vector<Waypoint> & map_waypoints)
{
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    string line;
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        Waypoint waypoint;
        iss >> waypoint.x;
        iss >> waypoint.y;
        iss >> waypoint.s;
        iss >> waypoint.dx;
        iss >> waypoint.dy;
        map_waypoints.push_back(waypoint);
    }
}


int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<Waypoint> map_waypoints;
    ReadWaypoints(map_waypoints);

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    State state;
    bool use_spline = false;

    h.onMessage([&map_waypoints, &state, &use_spline]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                 uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        if( length <=2 || data[0] != '4' || data[1] != '2' )
        {
            return;
        }
        auto s = hasData(data);
        if ( s== "")
        {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
        }


        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
            // j[1] is the data JSON object

            // Main car's localization Data
            Car car;

            car.x = j[1]["x"];
            car.y = j[1]["y"];
            car.s = j[1]["s"];
            car.d = j[1]["d"];
            car.yaw = deg2rad(j[1]["yaw"]);
            car.speed = j[1]["speed"];

            // Previous path data given to the Planner
            vector<double> previous_path_x = j[1]["previous_path_x"];
            vector<double> previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side
            //   of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            int path_size = previous_path_x.size();

            if(path_size > 0)
            {
                car.s = end_path_s;
            }
            else if( car.speed == 0)
            {
                //restart case
                state.ref_speed = 0;
                state.SetLane(car.d);

            }

            bool too_close = false;

            VehicleController controller;
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
                Sensor s;
                s.d = sensor_fusion[i][6];
                s.vx = sensor_fusion[i][3];
                s.vy = sensor_fusion[i][4];
                s.s = sensor_fusion[i][5];
                controller.add_sensor(s, car);
            }
            int curr = state.lane;
            controller.update_lane(car, state, path_size);
            state.prev = curr;
            //finish change lane
            if( get_lane(car.d) != get_lane(end_path_d))
            {
                state.lane = curr;
            }

            Car pos;
            pos.x = car.x;
            pos.y = car.y;
            pos.s = car.s;
            pos.yaw = car.yaw;
            vector<Point>  new_points;

            tk::spline s;
            ConstructSpline(s, path_size, car, pos, map_waypoints, previous_path_x, previous_path_y, state.lane);
            generete_points(path_size, new_points, pos, s, state.ref_speed);


            //use_spline = !use_spline;

            vector<double> next_x_vals;
            vector<double> next_y_vals;
            copy_elements(next_x_vals, next_y_vals, previous_path_x, previous_path_y);
            copy_elements(next_x_vals, next_y_vals, new_points);


            json msgJson;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if


        // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
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
