#include "WPILib.h"
#include "pthread.h"
#include "Constants.cpp"
#include "Pickup.h"
#include "Switch.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{

	RobotDrive robotDrive;	// robot drive system
	I2C i2c;
	Joystick driveStick;	// joystick to operate drive
	Joystick grabStick;	// joystick to operate arm
	AnalogInput prox;	// ultrasonic proximity sensor
	Talon grabTalon;//motor controller for grabber
	Switch grabInnerLimit;//controls at what point grabber should stop
	Switch grabOuterLimit;//contrls at what point grabber should stop
	Talon liftTalon;//motor controller for lift
	Encoder liftEncoder;//tells how high the lift is
	Switch liftInnerLimit;//stops at the very top or at the very bottom of the lift
	Switch liftOuterLimit;//stops at the very top or at the very bottom of the lift
	PowerDistributionPanel pdp;
	Pickup pickup;


public:
	Robot() :
		robotDrive(Constants::frontLeftPin, Constants::rearLeftPin, Constants::frontRightPin, Constants::rearRightPin),//tells the robot where everything is plugged in
		i2c(I2C::kOnboard,5),
		driveStick(Constants::driveStickChannel),
		grabStick(Constants::grabStickChannel),
		prox(Constants::ultrasonicPin),
		grabTalon(Constants::grabTalonPin),
		grabInnerLimit(Constants::grabInnerLimitPin),
		grabOuterLimit(Constants::grabOuterLimitPin),
		liftTalon(Constants::liftTalonPin),
		liftEncoder(Constants::liftEncoderAPin, Constants::liftEncoderBPin),
		liftInnerLimit(Constants::liftInnerLimitPin),
		liftOuterLimit(Constants::liftOuterLimitPin),
		pdp(),
		pickup(grabTalon, grabInnerLimit, grabOuterLimit, liftTalon, liftEncoder, liftInnerLimit, liftOuterLimit, pdp)
{
		robotDrive.SetExpiration(0.1);	// safety feature
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true); // make the motors go the right way
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
}
	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()//teleop code
	{
		robotDrive.SetSafetyEnabled(false);
		double height = 0; //variable for lifting thread
		uint8_t toSend[10];//array of bytes to send over I2C 
		uint8_t toReceive[10];//array of bytes to receive over I2C
		uint8_t numToSend = 1;//number of bytes to send
		uint8_t numToReceive = 0;//number of bytes to receive
		toSend[0] = 1;//set the byte to send to 1
		i2c.Transaction(toSend, 1, toReceive, 0);//send over I2C
		bool isGrabbing = false;//whether or not grabbing thread is running
		bool isLifting = false;//whether or not lifting thread is running

		while (IsOperatorControl() && IsEnabled())
		{
			// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input is set to zero.

			toSend[0] = 1;
			numToSend = 1;

			if (driveStick.GetRawButton(Constants::driveOneAxisButton)) {	// if the trigger is pressed
				toSend[0] = 2;
				if (fabs(driveStick.GetX()) > fabs(driveStick.GetY()) && fabs(driveStick.GetX()) > fabs(driveStick.GetZ())) {
					robotDrive.MecanumDrive_Cartesian(scaleJoysticks(driveStick.GetX(), Constants::xDeadZone, Constants::xMax, Constants::xDegree), 0, 0, 0);	// drive with filtered X-only input
				}
				else if (fabs(driveStick.GetY()) > fabs(driveStick.GetX()) && fabs(driveStick.GetY()) > fabs(driveStick.GetZ())) {
					robotDrive.MecanumDrive_Cartesian(0, scaleJoysticks(driveStick.GetY(), Constants::yDeadZone, Constants::yMax, Constants::yDegree), 0, 0);	// drive with filtered Y-only input
				}
				else {
					robotDrive.MecanumDrive_Cartesian(0, 0, scaleJoysticks(driveStick.GetZ(), Constants::zDeadZone, Constants::zMax, Constants::zDegree), 0);	// drive with filtered Z-only input
				}
			}
			else if (driveStick.GetRawButton(Constants::driveXYButton)) {	// if the thumb is pressed
				toSend[0] = 3;//set value to send to the arduino to 3
				numToSend = 1;//send 1 byte
				robotDrive.MecanumDrive_Cartesian(scaleJoysticks(driveStick.GetX(), Constants::xDeadZone, Constants::xMax, Constants::xDegree), scaleJoysticks(driveStick.GetY(),Constants::yDeadZone, Constants::yMax, Constants::yDegree), 0, 0);	// drive with filtered XY-only input
			}
			else if (driveStick.GetRawButton(5)){
				toSend[0] = 4;
				numToSend =1;
			}
			else if (driveStick.GetRawButton(3)){
				toSend[0] = 5;
				numToSend = 1;
			}
			else if (driveStick.GetRawButton(4)){
				toSend[0] = 6;
				numToSend = 1;
			}
			else {	// if no drive restricting buttons are pressed
				robotDrive.MecanumDrive_Cartesian(scaleJoysticks(driveStick.GetX(), Constants::xDeadZone, Constants::xMax, Constants::xDegree), scaleJoysticks(driveStick.GetY(),Constants::yDeadZone, Constants::yMax, Constants::yDegree), scaleJoysticks(driveStick.GetZ(),Constants::zDeadZone, Constants::zMax, Constants::zDegree), 0);	// drive with filtered input
			}
			pickup.setGrabber(scaleJoysticks(grabStick.GetX(), Constants::grabDeadZone, Constants::grabMax, Constants::grabDegree));
			pickup.setLifter(scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree));


			if (grabStick.GetRawButton(1)) {//if grab button is pressed
				pickup.grabberPosition(isGrabbing, grabStick);//start grabber thread
			}
			if (grabStick.GetRawButton(2)) {//if lift button is pressed
				pickup.lifterPosition(height, isLifting, grabStick);//start lifting thread
			}

			if (isGrabbing) {//if the grabbing thread is running
				toSend[0] = 7;//sends 7
				numToSend = 1;//sends 1 byte to I2C
			}

			float distance = prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12;	// distance from ultrasonic sensor
			float rotations = (float) liftEncoder.Get() / Constants::liftEncoderTicks;	// rotations on encoder
			SmartDashboard::PutNumber("Distance", distance);	// write stuff to smart dash
			SmartDashboard::PutNumber("Current", pdp.GetCurrent(Constants::grabPdpChannel));
			SmartDashboard::PutNumber("LED Current", pdp.GetCurrent(Constants::ledPdpChannel));
			SmartDashboard::PutNumber("Encoder", rotations);

			i2c.Transaction(toSend, 1, toReceive, 0);//send and receive information from arduino over I2C
			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
		toSend[0] = 0; //send 0 to arduino
		numToSend = 1;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);
	}

	void Autonomous()
	{
		float power = 0; 

		while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12 < 2) {	// while the nearest object is closer than 3 feet
		}

		while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12 < 12) {	// while the nearest object is closer than 3 feet
			if (power < .6) { //ramp up the power slowly
				power += .005;
			}
			robotDrive.MecanumDrive_Cartesian(0, power, 0);	// drive back
			Wait(.005);
		}

		for (int i = 0; i < 10; i++) //pause to make motors stop
		{
			robotDrive.MecanumDrive_Cartesian(0,0,0); ///STOP!!!
			Wait(.05);
		}

		power = 0;

		/*while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12 < 5) {	// while the nearest object is closer than 3 feet
			if (power < .6) {
				power += .005;
			}
			robotDrive.MecanumDrive_Cartesian(power, 0, 0);	// drive right
			Wait(.005);
		}

		for (int i = 0; i < 5; i++)
		{
			Wait(.05);
		}

		while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12 > 5) {	// while the nearest object is closer than 3 feet
			if (power < .6) {
				power += .005;
			}
			robotDrive.MecanumDrive_Cartesian(power, 0, 0);	// drive right
			Wait(.005);
		} */
		robotDrive.TankDrive(0.0, 0);	// STOP!!!
	}


	float scaleJoysticks(float power, float dead, float max, int degree) {	// scale joystick input to avoid crazy driving
		if (degree < 0) {	// make sure degree is positive
			degree = 1;
		}
		if (degree % 2 == 0) {	// make sure degree is odd
			degree++;
		}
		if (fabs(power) < dead) {	// if joystick input is in dead zone, return 0
			return 0;
		}
		else if  (power > 0) {	// if it is outside of the dead zone, then the output is a function of specified degree centered at the end of the dead zone
			return (max * pow(power - dead, degree) / pow(1 - dead, degree));
		}
		else {
			return (max * pow(power + dead, degree) / pow(1 - dead, degree));
		}
	}

};

START_ROBOT_CLASS(Robot);
