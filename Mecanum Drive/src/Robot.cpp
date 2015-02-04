#include "WPILib.h"
#include "Buttons/JoystickButton.h"
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
	Switch grabOuterLimit;//controls at what point grabber should stop
	Talon liftTalon;//motor controller for lift
	Encoder liftEncoder;//tells how high the lift is
	Switch liftInnerLimit;//stops at the very top or at the very bottom of the lift
	Switch liftOuterLimit;//stops at the very top or at the very bottom of the lift
	PowerDistributionPanel pdp;
	Gyro gyro;
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
		gyro(0),
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
		gyro.Reset();
		double height = 0; //variable for lifting thread
		uint8_t toSend[10];//array of bytes to send over I2C
		uint8_t toReceive[10];//array of bytes to receive over I2C
		uint8_t numToSend = 1;//number of bytes to send
		uint8_t numToReceive = 0;//number of bytes to receive
		toSend[0] = 1;//set the byte to send to 1
		i2c.Transaction(toSend, 1, toReceive, 0);//send over I2C
		bool isGrabbing = false;//whether or not grabbing thread is running
		bool isLifting = false;//whether or not lifting thread is running
		float driveX = 0;
		float driveY = 0;
		float driveZ = 0;
		float driveGyro = 0;

		while (IsOperatorControl() && IsEnabled())
		{
			// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input is set to zero.

			toSend[0] = 1;
			numToSend = 1;


			driveX = driveStick.GetRawAxis(Constants::driveXAxis);//starts driving code
			driveY = driveStick.GetRawAxis(Constants::driveYAxis);
			driveZ = driveStick.GetRawAxis(Constants::driveZAxis);
			driveGyro = gyro.GetAngle();


			uint32_t stickPtr = (uint32_t) (&driveStick);
			GenericHID *stick = (GenericHID *) stickPtr;
			JoystickButton trigger(stick, 5);
			SmartDashboard::PutNumber("Num Grab Buttons", grabStick.GetButtonCount());
			SmartDashboard::PutNumber("Num Drive Buttons", driveStick.GetButtonCount());


			if (driveStick.GetRawButton(Constants::driveOneAxisButton)) {//if X is greater than Y and Z, then it will only go in the direction of X
				toSend[0] = 3;
				numToSend = 1;

				if (fabs(driveX) > fabs(driveY) && fabs(driveX) > fabs(driveZ)) {
					driveY = 0;
					driveZ = 0;
				}
				else if (fabs(driveY) > fabs(driveX) && fabs(driveY) > fabs(driveZ)) {//if Y is greater than X and Z, then it will only go in the direction of Y
					driveX = 0;
					driveZ = 0;
				}
				else {//if Z is greater than X and Y, then it will only go in the direction of Z
					driveX = 0;
					driveY = 0;
				}
			}

			if (driveStick.GetRawButton(Constants::driveXYButton)) {//Z lock; only lets X an Y function
				driveZ = 0;//Stops Z while Z lock is pressed
			}

			if (driveStick.GetRawButton(Constants::driveFieldLockButton)) {//robot moves based on the orientation of the field
				driveGyro = 0;//gyro stops while field lock is enabled
			}

			driveX = scaleJoysticks(driveX, Constants::xDeadZone, Constants::xMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::xDegree);
			driveY = scaleJoysticks(driveY, Constants::yDeadZone, Constants::yMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::yDegree);
			driveZ = scaleJoysticks(driveZ, Constants::zDeadZone, Constants::zMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::zDegree);
			robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ, driveGyro);//makes the robot drive





			pickup.setGrabber(scaleJoysticks(grabStick.GetX(), Constants::grabDeadZone, Constants::grabMax, Constants::grabDegree));
			pickup.setLifter(scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree));


			if (grabStick.GetRawButton(Constants::grabButton)) {//if grab button is pressed
				pickup.grabberPosition(isGrabbing, grabStick);//start grabber thread
			}
			if (grabStick.GetRawButton(Constants::liftButton)) {//if lift button is pressed
				pickup.lifterPosition(height, isLifting, grabStick);//start lifting thread
			}

			if (isLifting) {//if the grabbing thread is running
				toSend[0] = 3;//sends 7
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
