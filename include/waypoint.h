#ifndef WAYPOINT_H
#define WAYPOINT_H


class Waypoint {
    public:
        double x;
        double y;
        double theta;

        // Default constructor with initial values
        Waypoint() : x(0.0), y(0.0), theta(0.0) {}


        Waypoint(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

#endif