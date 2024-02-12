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
#include "odom.h"
#include "waypoint.h"


//Macros & constants
#define DRIVE_MAX_SPEED 100
#define DRIVE_3QUARTER_SPEED 75
#define DRiVE_HALF_SPEED 50
#define DRIVE_QUARTER_SPEED 20
#define WHEEL_DIAMETER 4.0
#define ENCODER_TICKS_P_REV 900.0
#define CIRCUMFERENCE (WHEEL_DIAMETER * 3.1415926535)
#define TRACK_WIDTH 9.0

#define PI 3.1415926
#define RAD_DEG PI/180
#define DEG_RAD 180/PI


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
vex::motor motor_rarm(vex::PORT2);
vex::motor motor_primer(vex::PORT13);


//motor_group and drivetrain creation
vex::motor_group left_wheels(motor_lwheel);
vex::motor_group right_wheels(motor_rwheel);

//GET ACTUAL MEASUREMENTS FOR WHEELBASE (Distance from centerpoint to front axel) AND TRACKWIDTH (Distance between wheels)
vex::drivetrain w_robot(left_wheels, right_wheels, WHEEL_DIAMETER, TRACK_WIDTH, 9.5, vex::distanceUnits::in);




//odom
double l_encoder = 0;
double r_encoder = 0;

Waypoint current_pos = Waypoint();


void update_position(){
    double current_l_encoder = left_wheels.position(vex::rotationUnits::deg);
    double current_r_encoder = left_wheels.position(vex::rotationUnits::deg);


    double delta_left = current_l_encoder - l_encoder;
    double delta_right = current_r_encoder - r_encoder;
    double delta_s = 
    l_encoder = current_l_encoder;
    r_encoder = current_r_encoder;
}



void autonomous(){
    motor_primer.spinFor(vex::directionType::fwd, 0.5, vex::timeUnits::sec, 50, vex::velocityUnits::pct);
    motor_primer.spinFor(vex::directionType::rev, 0.5, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
}


void prime_launch(){
    motor_primer.spinFor(vex::directionType::fwd, 0.5, vex::timeUnits::sec, 50, vex::velocityUnits::pct);
    motor_primer.spinFor(vex::directionType::rev, 0.5, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
}

void opcontrol(){
    while(true) {
        int left_wheel_spin = ctrler.Axis3.position() * 0.75;
        int right_wheel_spin = ctrler.Axis2.position() * 0.75;


        //Prime the arm for throwing triballs
        ctrler.ButtonL1.pressed(prime_launch);

        // arms

        if (ctrler.ButtonR1.pressing()) {
            motor_rarm.spin(vex::directionType::fwd, 32, vex::velocityUnits::pct);

        } else if (ctrler.ButtonR2.pressing()) {
            motor_rarm.spin(vex::directionType::rev, 32, vex::velocityUnits::pct);

        } else {
            motor_rarm.stop();
        }
        //end arms

        if(left_wheel_spin == 0){
            left_wheels.setStopping(vex::brakeType::brake);
        }

        if(right_wheel_spin == 0){
            right_wheels.setStopping(vex::brakeType::brake);
        }


        if(ctrler.ButtonUp.pressing()){
            left_wheels.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            right_wheels.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        //drive
        left_wheels.spin(vex::directionType::fwd, left_wheel_spin, vex::velocityUnits::pct);
        right_wheels.spin(vex::directionType::fwd, right_wheel_spin, vex::velocityUnits::pct);
        this_thread::sleep_for(10);
    }
}

int main() {

    Brain.Screen.printAt( 10, 50, ";salfjas;ldfkjas;dlfkjas;dlfkjas;ldfjitsworkingwthasdfasdf-879809187()*&8567_*(7978056)" );
    

    Comp.autonomous(autonomous);
    Comp.drivercontrol(opcontrol);


    // //opcontrol testing
    // opcontrol();
    while(true){
        this_thread::sleep_for(10);
    }
   
}
