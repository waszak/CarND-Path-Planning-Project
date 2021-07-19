#ifndef CAR_H
#define CAR_H

struct Point
{
    double x;
    double y;
};

struct Waypoint:Point
{
    double s;
    double d;
    double dx;
    double dy;
};

struct Car:Point
{

    double s;
    double d;
    double yaw;
    double speed;

};
#endif
