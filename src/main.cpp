#include "main.h"
#include "pros/screen.hpp"
#include "robot-config.cpp"
#include <cstdlib>
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// Controller //

okapi::Controller master;


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




void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	std::cout << "Setting OKAPI log level \n";
	okapi::Logger::setDefaultLogger(
			std::make_shared<okapi::Logger>(
					okapi::TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
					"/ser/sout", // Output to the PROS terminal
					okapi::Logger::LogLevel::info // Show info, errors and warnings -- warn, debug, info
			)
	);



}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void motorBrake() {
	OkLeftFrontMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkLeftRearMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkRightFrontMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkRightRearMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
}

void autonomous() {
	
	
	std::shared_ptr<OdomChassisController> Chassis =

    ChassisControllerBuilder()
        .withMotors(OkLeftFrontMotor,OkRightFrontMotor, OkRightRearMotor, OkLeftRearMotor)
        .withDimensions(okapi::AbstractMotor::gearset::green,
                        {{0.1016_m, 9_in}, okapi::imev5GreenTPR})
        .withSensors(OkENCL, OkENCR)
        .withOdometry({{2.75_in, 9.75_in}, okapi::quadEncoderTPR},
                      okapi::StateMode::CARTESIAN)
		
		.withGains(
        {0.00122, 0.0000000000, 0.00001}, // distance controller gains
        {0.00286, 0.0000000000, 0.00001}, // turn controller gains
        {0.000000, 0, 0.00000}  // angle controller gains (helps drive straight
		)
		.withMaxVoltage(700)
        .buildOdometry();
	
std::shared_ptr<okapi::AsyncMotionProfileController> profileControllerF = okapi::AsyncMotionProfileControllerBuilder()
  .withLimits({ 1.06 * 0.9,  // Maximum linear velocity of the Chassis in m/s
                2.00 * 0.9,  // Maximum linear acceleration of the Chassis in m/s/s
               10.00 * 0.9}) // Maximum linear jerk of the Chassis in m/s/s/s
  .withOutput(Chassis) // Chassis Controller
  .buildMotionProfileController();

std::shared_ptr<okapi::AsyncMotionProfileController> profileControllerM = okapi::AsyncMotionProfileControllerBuilder()
  .withLimits({ 1.06 * 0.66,  // Maximum linear velocity of the Chassis in m/s
                2.00 * 0.66,  // Maximum linear acceleration of the Chassis in m/s/s
               10.00 * 0.66}) // Maximum linear jerk of the Chassis in m/s/s/s
  .withOutput(Chassis) // Chassis Controller
  .buildMotionProfileController();

std::shared_ptr<okapi::AsyncMotionProfileController> profileControllerS = okapi::AsyncMotionProfileControllerBuilder()
  .withLimits({ 1.06 * 0.4,  // Maximum linear velocity of the Chassis in m/s
                2.00 * 0.4,  // Maximum linear acceleration of the Chassis in m/s/s
               10.00 * 0.4}) // Maximum linear jerk of the Chassis in m/s/s/s
  .withOutput(Chassis) // Chassis Controller
  .buildMotionProfileController();
	
	OkLeftFrontMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkLeftRearMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkRightFrontMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	OkRightRearMotor.setBrakeMode(AbstractMotor::brakeMode::brake);
	
	Chassis->setState({0_m,0_m,0_deg});
	std::cout << "Setting starting position of 0,0,0 \n";

		\
		// for debugging purppose get encoder counts as well and show on console
		std::cout << "Encoder LEFT value: " << OkENCL.get() << " -- ";
	  std::cout << "RIGHT value: " << OkENCR.get() << "\n";

	
	profileControllerM->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
 	profileControllerM->setTarget("A");
 	profileControllerS->generatePath({{0_in, 0_in, 0_deg}, {32_in, 28_in, 0_deg}}, "B");
 	profileControllerM->waitUntilSettled();

 	profileControllerS->setTarget("B", true); // true means to use the path in reverse
 	profileControllerM->removePath("A");
 	profileControllerM->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "C");
 	profileControllerS->waitUntilSettled();
 	profileControllerM->setTarget("C");
	
	//Chassis->moveDistance(6_ft);
	//motorBrake();
	//Chassis->waitUntilSettled();
	//Chassis->driveToPoint({2_ft,3_ft});                                                                                                             
	/*Chassis->moveDistance(2_ft);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->turnAngle(90_deg);
	Chassis->waitUntilSettled();
	motorBrake();

	Chassis->moveDistance(2_ft);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->turnAngle(90_deg);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->moveDistance(2_ft);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->turnAngle(90_deg);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->moveDistance(2_ft);
	Chassis->waitUntilSettled();
	motorBrake();
	
	Chassis->turnAngle(90_deg);
	Chassis->waitUntilSettled();
	motorBrake();

	*/

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	//pros::Motor left_mtr(1);
	//pros::Motor right_mtr(2);
/*
	 std::shared_ptr<ChassisController> drive =

    ChassisControllerBuilder()
        .withMotors(OkLeftFrontMotor, OkRightFrontMotor,OkRightRearMotor,OkLeftRearMotor)
        .withDimensions(okapi::AbstractMotor::gearset::green,
                        {{0.1016_m, 0.1397_m}, okapi::imev5GreenTPR})
        .withSensors(OkENCL, OkENCR)
        .withOdometry({{0.06985_m, 0.06985_m}, okapi::quadEncoderTPR},
                      okapi::StateMode::FRAME_TRANSFORMATION)

        .build();
*/
pros::Controller prosMaster(pros::E_CONTROLLER_MASTER);
pros::Motor LeftFrontMotor(2,MOTOR_GEARSET_18);
pros::Motor LeftRearMotor(18,MOTOR_GEARSET_18);
pros::Motor RightFrontMotor(7,MOTOR_GEARSET_18,true);
pros::Motor RightRearMotor(19,MOTOR_GEARSET_18,true);

while(true){

	//drive->getModel()->XDriveModel(master.getAnalog(ControllerAnalog::leftY),
						  //master.getAnalog(ControllerAnalog::leftX));













		LeftFrontMotor.move(prosMaster.get_analog(ANALOG_LEFT_Y) + prosMaster.get_analog(ANALOG_RIGHT_X) + prosMaster.get_analog(ANALOG_LEFT_X));
	    RightFrontMotor.move(prosMaster.get_analog(ANALOG_LEFT_Y) - prosMaster.get_analog(ANALOG_RIGHT_X) - prosMaster.get_analog(ANALOG_LEFT_X));
	    LeftRearMotor.move(prosMaster.get_analog(ANALOG_LEFT_Y) + prosMaster.get_analog(ANALOG_RIGHT_X) - prosMaster.get_analog(ANALOG_LEFT_X));
	    RightRearMotor.move(prosMaster.get_analog(ANALOG_LEFT_Y) - prosMaster.get_analog(ANALOG_RIGHT_X) + prosMaster.get_analog(ANALOG_LEFT_X));


		std::cout << "Encoder LEFT value: " << OkENCL.get() << " -- ";
	  std::cout << "RIGHT value: " << OkENCR.get() << "\n";

		/*
		double front_Left  = (double)(master.getAnalog(okapi::ControllerAnalog::leftY) + master.getAnalog(okapi::ControllerAnalog::leftX));
    	double back_Left   = (double)(master.getAnalog(okapi::ControllerAnalog::leftY) - master.getAnalog(okapi::ControllerAnalog::leftX));
    	double front_Right = (double)(master.getAnalog(okapi::ControllerAnalog::leftY) - master.getAnalog(okapi::ControllerAnalog::leftX));
		double back_Right  = (double)(master.getAnalog(okapi::ControllerAnalog::leftY) + master.getAnalog(okapi::ControllerAnalog::leftX));

		double max_raw_sum = (double)(abs(master.getAnalog(okapi::ControllerAnalog::leftY)) + abs(master.getAnalog(okapi::ControllerAnalog::leftX)));

		double max_XYstick_value = (double)(std::max(abs(master.getAnalog(okapi::ControllerAnalog::leftY)),abs(master.getAnalog(okapi::ControllerAnalog::leftX))));

		if (max_raw_sum != 0) {
      front_Left  = front_Left / max_raw_sum * max_XYstick_value;
      back_Left   = back_Left / max_raw_sum * max_XYstick_value;
      front_Right = front_Right / max_raw_sum * max_XYstick_value;
      back_Right  = back_Right / max_raw_sum * max_XYstick_value;
    	}

		front_Left  = front_Left  + master.getAnalog(okapi::ControllerAnalog::rightX);
    back_Left   = back_Left   + master.getAnalog(okapi::ControllerAnalog::rightX);
    front_Right = front_Right - master.getAnalog(okapi::ControllerAnalog::rightX);
    back_Right  = back_Right  - master.getAnalog(okapi::ControllerAnalog::rightX);

		max_raw_sum = std::max(std::abs(front_Left),std::max(std::abs(back_Left),std::max(std::abs(front_Right),std::max(std::abs(back_Right),100.0))));

		front_Left  = front_Left  / max_raw_sum * 100.0;
    back_Left   = back_Left   / max_raw_sum * 100.0;
    front_Right = front_Right / max_raw_sum * 100.0;
    back_Right  = back_Right  / max_raw_sum * 100.0;

		OkLeftFrontMotor.moveVelocity(front_Left);
    OkLeftRearMotor.moveVelocity(back_Left);
    OkRightFrontMotor.moveVelocity(front_Right);
    OkRightRearMotor.moveVelocity(back_Right );

printf(" Left_Front %f /n",OkLeftFrontMotor.getActualVelocity());
printf("Left_FrontPWR %f /n",front_Left);
printf(" Left_Rear %f /n",OkLeftRearMotor.getActualVelocity());
printf("Left_RearPWR %f /n",back_Left);
printf(" Right_Front %f /n",OkRightFrontMotor.getActualVelocity());
printf("Right_FrontPWR %f /n",front_Right);
printf(" Right_Rear %f /n",OkRightRearMotor.getActualVelocity());
printf("Right_RearPWR %f /n",back_Right);

printf("%f",master.getAnalog(okapi::ControllerAnalog::leftY));
*/
pros::delay(10);
}

}
