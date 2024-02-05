/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ewu                                                       */
/*    Created:      1/10/2024, 7:57:08 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <math.h>



//Macros & constants
#define DRIVE_MAX_SPEED 100
#define DRIVE_3QUARTER_SPEED 75
#define DRiVE_HALF_SPEED 50
#define DRIVE_QUARTER_SPEED 20
#define WHEEL_DIAMETER 4.0
#define ENCODER_TICKS_P_REV 900.0
#define CIRCUMFERENCE (WHEEL_DIAMETER * 3.1415926535)
#define TRACK_WIDTH 8.0


using namespace vex;

//Waypoint structure
struct Waypoint {
    double x;
    double y;
};

Waypoint waypoints[4];
int idx = 0;
//initialize vex components
vex::brain Brain;
vex::competition Comp;
vex::controller ctrler = vex::controller();

vex::motor motor_lwheel(vex::PORT11, ratio18_1, false);
vex::motor motor_rwheel(vex::PORT1, ratio18_1, true);
vex::motor motor_larm(vex::PORT12);
vex::motor motor_rarm(vex::PORT2);
vex::motor motor_lprimer(vex::PORT13);
vex::motor motor_rprimer(vex::PORT3);

//motor_group and drivetrain creation
vex::motor_group left_wheels(motor_lwheel);
vex::motor_group right_wheels(motor_rwheel);

//GET ACTUAL MEASUREMENTS FOR WHEELBASE (Distance from centerpoint to front axel) AND TRACKWIDTH (Distance between wheels)
vex::drivetrain w_robot(left_wheels, right_wheels, WHEEL_DIAMETER, TRACK_WIDTH, 5.0, vex::distanceUnits::in);

//dead reckoning
double previous_left_encoder = 0.0; //previous encoder for left motor group
double previous_right_encoder = 0.0; //previous encoder for right motor group
double x = 0.0; // current x-coordinate
double y = 0.0; // current y-coordinate
double theta = 0.0; //angle in degrees


void update_pos(){
    double left_encoder = motor_lwheel.position(vex::rotationUnits::deg);
    double right_encoder = motor_rwheel.position(vex::rotationUnits::deg);

    double delta_left = left_encoder - previous_left_encoder;
    double delta_right = right_encoder - previous_right_encoder;

    // Convert encoder ticks to inches
    delta_left = (delta_left / ENCODER_TICKS_P_REV) * (CIRCUMFERENCE);
    delta_right = (delta_right / ENCODER_TICKS_P_REV) * (CIRCUMFERENCE);

    // Update the previous encoder values
    previous_left_encoder = left_encoder;
    previous_right_encoder = right_encoder;

    // Compute robot displacement
    double delta_s = (delta_left + delta_right) / 2.0;
    double delta_theta = (delta_right - delta_left) / TRACK_WIDTH;

    // Update x, y, and theta
    x = x + delta_s * cos(theta + delta_theta / 2.0);
    y = y + delta_s * sin(theta + delta_theta / 2.0);
    theta = theta + delta_theta;

    //if theta >= 360 or theta < 0, then update oreintation to theta mod 360
    theta = fmod(theta, 360.0);
}


void goto_waypoint(int loc){

}

void return_to_sender(){
    update_pos();

    double rotation_amount = theta > 180 ? (360.0 - theta) : -1.0 * theta;
    double travel_dist = sqrt(pow(x, 2) + pow(y, 2));

    w_robot.turnFor(rotation_amount, vex::rotationUnits::deg, false);
    w_robot.driveFor(travel_dist, vex::distanceUnits::in, false);

    //saftey condition
    while (true) {
        if (ctrler.ButtonX.pressing()) {
            w_robot.stop();
            return;  // Exit for saftey
        }
        this_thread::sleep_for(10);
    }
}

void autonomous() {
    w_robot.driveFor(60.0, vex::distanceUnits::in);
    w_robot.turnFor(90.0, vex::rotationUnits::deg);

    motor_lprimer.spinFor(vex::directionType::fwd, 5.0, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
    motor_rprimer.spinFor(vex::directionType::fwd, 5.0, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
    motor_lprimer.setStopping(vex::brakeType::coast);
    motor_rprimer.setStopping(vex::brakeType::coast);

    while(true){
        this_thread::sleep_for(10);
        Brain.Screen.printAt( 10, 50, "Communism is overrated" );
    }
}

void opcontrol(){
    while(true) {
        int left_wheel_spin = ctrler.Axis3.position() * 5;
        int right_wheel_spin = ctrler.Axis2.position() * 5;


        //Prime the arm for throwing triballs
        if(ctrler.ButtonA.pressing()) {
            //calibrate force as necessary
            motor_lprimer.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
            motor_rprimer.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
        } else {
            motor_lprimer.setStopping(vex::brakeType::coast);
            motor_rprimer.setStopping(vex::brakeType::coast);
        }



        if(ctrler.ButtonY.pressing()){
            return_to_sender();
        }

        if(ctrler.ButtonB.pressing()){
            Waypoint temp = { x, y };
            waypoints[idx] = temp;
            idx++;
        }

        

        // arms
        if (ctrler.ButtonL1.pressing()) {
            motor_larm.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);

        } else if (ctrler.ButtonL2.pressing()) {
            motor_larm.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);

        } else {
            motor_larm.stop();
        }

        if (ctrler.ButtonR1.pressing()) {
            motor_rarm.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);

        } else if (ctrler.ButtonR2.pressing()) {
            motor_larm.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);

        } else {
            motor_rarm.stop();
        }
        //end arms


        //drive
        left_wheels.spin(vex::directionType::fwd, left_wheel_spin, vex::velocityUnits::pct);
        right_wheels.spin(vex::directionType::fwd, right_wheel_spin, vex::velocityUnits::pct);
        this_thread::sleep_for(10);
    }
}

int main() {

    Brain.Screen.printAt( 10, 50, ";salfjas;ldfkjas;dlfkjas;dlfkjas;ldfjitsworkingwthasdfasdf-879809187()*&8567_*(7978056)" );
    
    // use this for competition
    // Comp.autonomous(autonomous);
    // Comp.drivercontrol(opcontrol);




    //opcontrol testing
    opcontrol();
    while(true){
        this_thread::sleep_for(10);
    }
   
}
