#ifndef ODOM_H
#define ODOM_H
#include "waypoint.h"


void move_to(double x, double y);
void turn_to(double degrees); //degs are field centric not robot centric
void update_position();


#endif