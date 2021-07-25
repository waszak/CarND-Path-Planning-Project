#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H
#include <vector>
#include <algorithm>
using std::string;
using std::vector;
struct State
{
    int prev =1;
    int lane = 1;
    double ref_speed = 0;

    void SetLane(double d)
    {
        if(d<4)
        {
            lane = 0;
        }
        else if (d < 8)
        {
            lane = 1;
        }
        else
        {
            lane = 2;
        }
    }

};

struct Sensor
{
    double vx;
    double vy;
    double s;
    double d;

    bool operator<(Sensor const& sensor)
    {
        return s < sensor.s;
    }
};

struct VehicleController
{
    vector<Sensor> left_lane;
    vector<Sensor> center_lane;
    vector<Sensor> right_lane;

    vector<Sensor>& current_lane(double d)
    {
        if( d <4)
            return left_lane;
        else if (d<8)
            return center_lane;
        return right_lane;
    }

    vector<Sensor>& current_lane(State state)
    {
        if( state.lane == 0)
            return left_lane;
        else if (state.lane == 1)
            return center_lane;
        return right_lane;

    }

    void add_sensor(Sensor sensor, Car car)
    {
        double max_s = 6945.554 - 30;
        auto & lane = current_lane(sensor.d);

        //special case when 0
        if( car.s > max_s && sensor.s<100)
        {
            sensor.s += max_s + 30;
        }

        if(car.s - sensor.s<30 && sensor.s - car.s <100)
        {
            lane.push_back(sensor);
        }
    }

    bool has_collision(Car car, State state, int path_size, bool same_line)
    {
        bool too_close = false;
        auto & lane = current_lane(state);
        for(auto obj: lane)
        {
            double speed = distance(obj.vx, obj.vy);
            double s = obj.s;
            s += path_size * .02 * speed;


            if( same_line && s > car.s && s - car.s < 30)
            {
                too_close = true;
                break;
            }
            if(!same_line && ((car.s - s  < 10) && ( s - car.s < 30)) )//-30
            {
                too_close = true;
                break;
            }
        }
        return too_close;
    }

    double cost_lane(Car car, State& state)
    {
        int total = 0;
        double speed =0;
        auto & lane = current_lane(state);
        for(auto obj: lane)
        {
            if(obj.s < car.s)
            {
                continue;
            }
            if(obj.s - car.s >90)
            {
                break;
            }
            speed += distance(obj.vx, obj.vy);
            if(total > 2)
            {
                break;
            }
        }
        if(total == 0)
        {
            return 0;
        }
        else
        {
            return -speed/total;
        }
    }

    double speed_lane(Car car, State& state)
    {
        int total = 0;
        double speed =0;
        auto & lane = current_lane(state);
        for(auto obj: lane)
        {
            if(obj.s < car.s)
            {
                continue;
            }
            return -distance(obj.vx, obj.vy);
        }
        return 0;
    }

    void update_lane(Car car, State &state, int path_size)
    {
        //for easier processing.
        sort(left_lane.begin(), left_lane.end());
        sort(center_lane.begin(), center_lane.end());
        sort(right_lane.begin(), right_lane.end());

        bool collision = has_collision(car, state, path_size, true);
        double speed = speed_lane(car, state);

        if(!collision && state.ref_speed< 49.7)
        {
            state.ref_speed += 0.220;
        }
        else if (collision && state.ref_speed > speed+1)
        {
            state.ref_speed -= 0.220;
        }

        //if central lane is empty then switch
        if( state.lane != 1 && center_lane.size() == 0)
        {
            state.lane = 1;
            return;
        }
        if(!collision)
        {
            return;
        }


        State center, left, right;
        center.lane = 1;
        left.lane = 0;
        right.lane = 2;
        double cost_center = speed_lane(car, center);
        double cost_left = speed_lane(car, left);
        double cost_right = speed_lane(car, right);

        if(state.lane ==0 && cost_right == 0  && !has_collision(car, center, path_size, false))
        {
            state.lane = 1;
            return;
        }
        else if(state.lane == 2 && cost_left == 0 && !has_collision(car, center, path_size, false))
        {
            state.lane = 1;
            return;
        }

        if(state.lane != 1 && !has_collision(car, center, path_size, false))
        {
            state.lane = 1;
        }

        else if( state.lane == 1 )
        {
            bool left_check = !has_collision(car, left, path_size, false);
            bool right_check = !has_collision(car, right, path_size, false);

            if( left_lane.size() == 0||(left_check && !right_check))
            {
                state.lane = 0;
            }
            else if(right_lane.size() == 0 || (right_check && !left_check))
            {
                state.lane = 2;
            }
            else if (left_check && right_check )
            {
                if(cost_left < cost_right)
                {
                    state.lane = 0;
                }
                else
                {
                    state.lane = 2;
                }
            }
            /*else
            {
                //state.ref_speed = speed_lane(car, state);
                state.ref_speed -=.220;
                //std::cout<<state.ref_speed<<std::endl;
            }*/
        }
        /*else
        {
            //state.ref_speed = speed_lane(car, state);
            state.ref_speed -=.220;
            //std::cout<<state.ref_speed<<std::endl;
        }*/
    }
};

#endif
