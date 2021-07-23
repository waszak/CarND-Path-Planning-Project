#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "car.h"

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
struct state
{
    int lane = 1;
    double ref_speed = 0;

};
int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<Waypoint> map_waypoints;
    ReadWaypoints(map_waypoints);

    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    state state;


    h.onMessage([&map_waypoints, &state]
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
                if(car.d<4)
                {
                    state.lane = 0;
                }
                else if (car.d < 8)
                {
                    state.lane = 1;
                }
                else
                {
                    state.lane = 2;
                }

            }

            bool too_close = false;

            vector<int> left_lane;
            vector<int> center_lane;
            vector<int> right_lane;
            for(int i = 0; i < sensor_fusion.size(); i++)
            {
                float d = sensor_fusion[i][6];

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_car_s = sensor_fusion[i][5];
                double speed = distance(vx, vy);

                check_car_s += path_size * .02 * speed;

                if( check_car_s < car.s )
                {
                    continue;
                }
                if( d <4)
                {
                    left_lane.push_back(i);
                }
                else if (d>=4 && d<8)
                {
                    center_lane.push_back(i);
                }
                else if(d>=8)
                {
                    right_lane.push_back(i);
                }
            }
            vector<int> current_lane;
            if( state.lane == 0)
            {
                current_lane = left_lane;
            }
            else if (state.lane == 1)
            {
                current_lane = center_lane;
            }
            else
            {
                current_lane = right_lane;
            }

            /*if(center_lane.size() == 0)
            {
                lane = 1;
            }
            else if(left_lane.size() == 0)
            {
                lane = 0;
            }
            else if (right_lane.size()==0)
            {
                lane = 2;
            }*/
            for(int j = 0; j < current_lane.size(); j++)
            {
                int i = current_lane[j];


                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_car_s = sensor_fusion[i][5];
                double speed = distance(vx, vy);

                check_car_s += path_size * .02 * speed;

                if( check_car_s > car.s && check_car_s - car.s < 30)
                {
                    too_close = true;
                    break;
                }
            }
            if( too_close)
            {
                bool left_ok=true;
                if( state.lane == 1)
                {

                    for(int j = 0; j < left_lane.size(); j++)
                    {
                        int i = left_lane[j];


                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_car_s = sensor_fusion[i][5];
                        double speed = distance(vx, vy);

                        check_car_s += path_size * .02 * speed;

                        if( check_car_s > car.s && check_car_s - car.s < 30)
                        {
                            left_ok = false;
                            break;
                        }
                    }
                    if(left_ok)
                    {
                        state.lane -=1;
                    }

                    if(!left_ok)
                    {
                        bool right_ok = true;
                        for(int j = 0; j < right_lane.size(); j++)
                        {
                            int i = right_lane[j];


                            double vx = sensor_fusion[i][3];
                            double vy = sensor_fusion[i][4];
                            double check_car_s = sensor_fusion[i][5];
                            double speed = distance(vx, vy);

                            check_car_s += path_size * .02 * speed;

                            if( check_car_s > car.s && check_car_s - car.s < 30)
                            {
                                right_ok = false;
                                break;
                            }
                        }
                        if(right_ok)
                        {
                            state.lane +=1;
                        }
                    }
                }
                else if(state.lane == 0 || state.lane ==2)
                {
                    //center

                    bool center_ok = true;
                    for(int j = 0; j < center_lane.size(); j++)
                    {
                        int i = center_lane[j];


                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_car_s = sensor_fusion[i][5];
                        double speed = distance(vx, vy);

                        check_car_s += path_size * .02 * speed;

                        if( check_car_s > car.s && check_car_s - car.s < 30)
                        {
                            center_ok = false;
                            break;
                        }
                    }
                    if(center_ok)
                    {
                        state.lane = 1;
                    }
                }
            }

            if( too_close)
            {
                state.ref_speed -= 0.220;
            }
            else if (state.ref_speed< 49.7)
            {
                state.ref_speed += 0.220;
            }

            Car pos;
            pos.x = car.x;
            pos.y = car.y;
            pos.s = car.s;
            pos.yaw = car.yaw;

            tk::spline s;
            ConstructSpline(s, path_size, car, pos, map_waypoints, previous_path_x, previous_path_y, state.lane);

            vector<Point>  new_points;
            generete_points(path_size, new_points, pos, s, state.ref_speed);

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
