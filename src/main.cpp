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
#define TRACK_WIDTH 9.0


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



void autonomous() {
    w_robot.driveFor(24.0, vex::distanceUnits::in);
    w_robot.turnFor(90.0, vex::rotationUnits::deg);
    w_robot.driveFor(18.0, vex::distanceUnits::in);
    w_robot.turnFor(-30.0, vex::rotationUnits::deg);
    motor_primer.spinFor(vex::directionType::fwd, 2.0, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
    motor_primer.spinFor(vex::directionType::rev, 1.5, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
    


    while(true){
        this_thread::sleep_for(10);
        Brain.Screen.printAt( 10, 50, "Communism is overrated" );
    }
}


void prime_launch(){
    motor_primer.spinFor(vex::directionType::fwd, 1.5, vex::timeUnits::sec, 50, vex::velocityUnits::pct);
    motor_primer.spinFor(vex::directionType::rev, 1.5, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
}

void opcontrol(){
    while(true) {
        int left_wheel_spin = ctrler.Axis3.position() * 0.75;
        int right_wheel_spin = ctrler.Axis2.position() * 0.75;


        //Prime the arm for throwing triballs
        ctrler.ButtonL1.pressed(prime_launch);


        ctrler.ButtonB.pressed(autonomous);

        // arms

        if (ctrler.ButtonR1.pressing()) {
            motor_rarm.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);

        } else if (ctrler.ButtonR2.pressing()) {
            motor_rarm.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);

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
