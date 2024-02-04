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

const double wheel_diameter = 4.0;
const double encoder_ticks_p_rev = 900.0;


using namespace vex;
vex::brain Brain;
vex::competition Comp;
vex::controller ctrler = vex::controller();

vex::motor motor_lwheel(vex::PORT11, ratio18_1, false);
vex::motor motor_rwheel(vex::PORT1, ratio18_1, true);
vex::motor motor_larm(vex::PORT12);
vex::motor motor_rarm(vex::PORT2);
vex::motor motor_primer(vex::PORT13);

vex::motor_group left_wheels(motor_lwheel);
vex::motor_group right_wheels(motor_rwheel);
//GET ACTUAL MEASUREMENTS FOR WHEELBASE (Distance from centerpoint to front axel) AND TRACKWIDTH (Distance between wheels)
vex::drivetrain w_robot(left_wheels, right_wheels, 4.0, 5.0, 5.0, vex::distanceUnits::in);

//work in progress... do we even need this??????
// bool bump_detection(){
//     int motor_lwheel_pos = motor_lwheel.position(vex::rotationUnits::deg);
//     int motor_rwheel_pos = motor_rwheel.position(vex::rotationUnits::deg);

//     if(motor_lwheel_pos)
// }


void return_to_sender(){
        if(ctrler.ButtonX.pressing()){
            return;
        }
}

void autonomous() {
    w_robot.driveFor(60.0, vex::distanceUnits::in);
    w_robot.turnFor(90.0, vex::rotationUnits::deg);

    motor_primer.spinFor(vex::directionType::fwd, 5.0, vex::timeUnits::sec, 100, vex::velocityUnits::pct);
    motor_primer.setStopping(vex::brakeType::coast);

    while(true){
        this_thread::sleep_for(10);
        Brain.Screen.printAt( 10, 50, "Communism is overrated" );
    }
}

void opcontrol(){
 while(true) {
        int left_wheel = ctrler.Axis3.position() * 5;
        int right_wheel = ctrler.Axis2.position() * 5;


        //Prime the arm for throwing triballs
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
        left_wheels.spin(vex::directionType::fwd, left_wheel, vex::velocityUnits::pct);
        right_wheels.spin(vex::directionType::fwd, right_wheel, vex::velocityUnits::pct);
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
