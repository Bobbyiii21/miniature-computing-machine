#include "main.h"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
//using namespace pros;


using namespace okapi;
#include <iostream>       // For file system support in C++ ability to write to file stream
#include <fstream>
#include <chrono>         // for tiem support - NOTE V5 has no date/time support!
#include <ctime>
// Ports //



/*


// Controller //
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors //

//pros::Motor LeftFrontMotor(LeftFrontPort,MOTOR_GEARSET_18);
//pros::Motor LeftRearMotor(LeftRearPort,MOTOR_GEARSET_18);
//pros::Motor RightFrontMotor(RightFrontPort,MOTOR_GEARSET_18,true);
//pros::Motor RightRearMotor(RightRearPort,MOTOR_GEARSET_18,true);

okapi::Motor OkLeftFrontMotor(2);
okapi::Motor OkLeftRearMotor(18);
okapi::Motor OkRightFrontMotor(-7);
okapi::Motor OkRightRearMotor(-19);

okapi::MotorGroup LeftMotors({OkLeftFrontMotor,OkLeftRearMotor});
okapi::MotorGroup RightMotors({OkRightRearMotor,OkRightFrontMotor});

// Sensors and Encoders //

//pros::Imu Inertial(inertial);
//pros::ADIEncoder ENCL(EncoderLTop,EncoderLBottom);
//pros::ADIEncoder ENCR(EncoderRTop,EncoderRBottom);
okapi::IMU Inertial(25);
okapi::ADIEncoder OkENCL({'G','H'});
okapi::ADIEncoder OkENCR({'C','D'});

std::shared_ptr<OdomChassisController> odomChassis =

    ChassisControllerBuilder()
        .withMotors(LeftMotors, RightMotors)
        .withDimensions(okapi::AbstractMotor::gearset::green,
                        {{0.1016_m, 0.1397_m}, okapi::imev5GreenTPR})
        .withSensors(OkENCL, OkENCR)
        .withOdometry({{0.06985_m, 0.06985_m}, okapi::quadEncoderTPR},
                      okapi::StateMode::FRAME_TRANSFORMATION)

        .buildOdometry();
        */