/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ewu                                                       */
/*    Created:      1/10/2024, 7:57:08 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

#define DRIVE_MAX_SPEED 100;
#define TURN_VELOCITY_SPEED 100;


using namespace vex;
vex::brain Brain;
vex::competition Comp;
vex::controller ctrler = vex::controller();

vex::motor motor_lwheel(vex::PORT11);
vex::motor motor_rwheel(vex::PORT1);
vex::motor motor_larm(vex::PORT12);
vex::motor motor_rarm(vex::PORT2);
vex::motor motor_primer(vex::PORT13);



int main() {

    Brain.Screen.printAt( 10, 50, ";salfjas;ldfkjas;dlfkjas;dlfkjas;ldfj" );
   
    while(true) {
        int left_wheel = ctrler.Axis3.position() * 5;
        int right_wheel = ctrler.Axis2.position() * 5;

        if(ctrler.ButtonA.pressing()) {
            //calibrate force as necessary
            motor_primer.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
        } else {
            motor_primer.setStopping(vex::brakeType::coast);
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
        motor_lwheel.spin(vex::directionType::fwd, left_wheel, vex::velocityUnits::pct);
        motor_rwheel.spin(vex::directionType::fwd, right_wheel, vex::velocityUnits::pct);
        this_thread::sleep_for(10);
    }
}
