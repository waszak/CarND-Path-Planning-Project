#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include "car.h"
// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return M_PI;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline double distance(double x, double y)
{
    return sqrt(x*x+y*y);
}

inline double rotate_x(double shift_x, double shift_y, double yaw)
{
    return shift_x * cos(yaw) - shift_y * sin(yaw);
}

inline double rotate_y(double shift_x, double shift_y, double yaw)
{
    return shift_x * sin(yaw) + shift_y * cos(yaw);
}

inline void copy_elements(vector<double> & next_x_vals,  vector<double> & next_y_vals, vector<double> & previous_path_x,  vector<double> & previous_path_y)
{
    int v_size = previous_path_x.size();
    for (int i = 0; i < v_size; ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
}

inline void copy_elements(vector<double> & next_x_vals,  vector<double> & next_y_vals, vector<Point> & previous_path)
{
    int v_size = previous_path.size();
    for (int i = 0; i < v_size; ++i)
    {
        next_x_vals.push_back(previous_path[i].x);
        next_y_vals.push_back(previous_path[i].y);
    }
}


inline void normalize(vector<double> & pts_x, vector<double> & pts_y, const Car& pos)
{
    for(int i=0; i < pts_x.size(); i++)
    {
        //shifts car reference angles to 0
        double shift_x = (pts_x[i] - pos.x);
        double shift_y = (pts_y[i] - pos.y);
        pts_x[i] = rotate_x(shift_x, shift_y, -pos.yaw);
        pts_y[i] = rotate_y(shift_x, shift_y, -pos.yaw);
    }
}

inline void denormalize(vector<Point> & pts, const Car& pos )
{
    for(int i=0; i < pts.size(); i++)
    {
        double x = pts[i].x;
        double y = pts[i].y;
        pts[i].x = rotate_x(x, y, pos.yaw) + pos.x;
        pts[i].y = rotate_y(x, y, pos.yaw) + pos.y;

    }
}

inline void generete_points(int path_size, vector<Point> & new_points, const Car& pos, tk::spline& s,  double ref_speed = 49.0)
{
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = distance(target_x, target_y);
    double N = target_dist/(0.02*ref_speed/2.24);

    double delta_x = target_x/N;
    double x_point = 0;
    int points = 30;
    //start of the simulator
    if(/*pos.s < 130 &&*/ ref_speed<20)
    {
        points = 10;
    }
    for (int i = 1; i < points-path_size; ++i)
    {
        /*if (ref_speed< 49.7)
        {
            ref_speed += 0.220;
            N = target_dist/(0.02*ref_speed/2.24);
            delta_x = target_x/N;

        }*/
        x_point +=  delta_x;
        Point point;
        point.x = x_point;
        point.y =  s(x_point);

        new_points.push_back(point);
    }

    denormalize(new_points, pos);
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = std::min(2*pi() - angle, angle);

    if (angle > pi()/2)
    {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<Waypoint> &maps)
{
    int prev_wp = -1;

    while (s > maps[prev_wp+1].s && (prev_wp < (int)(maps.size()-1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp+1)%maps.size();

    double heading = atan2((maps[wp2].y-maps[prev_wp].y),
                           (maps[wp2].x-maps[prev_wp].x));
    // the x,y,s along the segment
    double seg_s = (s-maps[prev_wp].s);

    double seg_x = maps[prev_wp].x+seg_s*cos(heading);
    double seg_y = maps[prev_wp].y+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}


inline void ConstructSpline(tk::spline & s, int path_size,const Car& car, Car &pos,
                            vector<Waypoint>& map_waypoints,
                            vector<double>& previous_path_x,
                            vector<double>& previous_path_y,
                            int lane)
{
    vector<double> pts_x;
    vector<double> pts_y;
    if(path_size < 2)
    {
        double prev_x = car.x - cos(car.yaw);
        double prev_y = car.y - sin(car.yaw);
        pts_x.push_back(prev_x);
        pts_y.push_back(prev_y);

        pts_x.push_back(car.x);
        pts_y.push_back(car.y);
    }
    else
    {
        //std::cout<<car.yaw<<std::endl;
        pos.x = previous_path_x[path_size-1];
        pos.y = previous_path_y[path_size-1];

        pts_x.push_back(previous_path_x[path_size-2]);
        pts_y.push_back(previous_path_y[path_size-2]);

        pts_x.push_back(pos.x);
        pts_y.push_back(pos.y);

        pos.yaw = atan2(pts_y[1]-pts_y[0], pts_x[1]-pts_x[0] );
    }

    for(int i=30; i<91; i+=30)
    {
        vector<double> xy = getXY(car.s+i, 2+4*lane, map_waypoints);
        pts_x.push_back(xy[0]);
        pts_y.push_back(xy[1]);
    }

    //shifts car reference angles to 0
    normalize(pts_x, pts_y, pos);

    //s.set_boundary(tk::spline::bd_type::second_deriv, pts_y[0],tk::spline::bd_type::second_deriv, pts_y[pts_y.size()-1]);
    //s.set_points(pts_x, pts_y);
    s.set_points(pts_x, pts_y,tk::spline::spline_type::cspline_hermite);
    //s.set_points(pts_x, pts_y,tk::spline::spline_type::linear);
}

#endif  // HELPERS_H
