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
	Timer timer; //timer
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
	Switch liftUpperLimit;//stops at top or bottom of the lift
	Switch liftLowerLimit;//stops at top or bottom of the lift
	Encoder grabEncoder;
	PowerDistributionPanel pdp;
	Gyro gyro;
	Pickup pickup;



public:
	Robot() :
		timer(),
		robotDrive(Constants::driveFrontLeftPin, Constants::driveRearLeftPin, Constants::driveFrontRightPin, Constants::driveRearRightPin),//tells the robot where everything is plugged in
		i2c(I2C::kOnboard,Constants::ledAddress),
		driveStick(Constants::driveStickChannel),
		grabStick(Constants::grabStickChannel),
		prox(Constants::driveUltrasonicPin),
		grabTalon(Constants::grabTalonPin),
		grabInnerLimit(Constants::grabInnerLimitPin),
		grabOuterLimit(Constants::grabOuterLimitPin),
		liftTalon(Constants::liftTalonPin),
		liftEncoder(Constants::liftEncoderAPin, Constants::liftEncoderBPin),
		liftUpperLimit(Constants::liftUpperLimitPin),
		liftLowerLimit(Constants::liftLowerLimitPin),
		grabEncoder(Constants::grabEncoderAPin, Constants::grabEncoderBPin),
		pdp(),
		gyro(Constants::driveGyroRate),
		pickup(grabTalon, grabInnerLimit, grabOuterLimit, liftTalon, liftEncoder, liftUpperLimit, liftLowerLimit, pdp)
{
		robotDrive.SetExpiration(0.1);	// safety feature
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true); // make the motors go the right way
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		liftEncoder.SetReverseDirection(Constants::liftEncoderReverse);
}
	/**
	 * Runs the motors with Mecanum drive.
	 */
	void OperatorControl()//teleop code
	{
		robotDrive.SetSafetyEnabled(false);
		gyro.Reset();
		grabEncoder.Reset();
		timer.Start();
		timer.Reset();
		double liftHeight = 0; //variable for lifting thread
		int liftHeightBoxes = 0; //another variable for lifting thread
		int liftStep = 0; //height of step in inches
		int liftRamp = 0; //height of ramp in inches
		double grabPower;
		bool backOut;
		uint8_t toSend[10];//array of bytes to send over I2C
		uint8_t toReceive[10];//array of bytes to receive over I2C
		uint8_t numToSend = 1;//number of bytes to send
		uint8_t numToReceive = 0;//number of bytes to receive
		toSend[0] = 1;//set the byte to send to 1
		i2c.Transaction(toSend, 1, toReceive, 0);//send over I2C
		bool isGrabbing = false;//whether or not grabbing thread is running
		bool isLifting = false;//whether or not lifting thread is running
		bool isBraking = false;//whether or not braking thread is running
		float driveX = 0;
		float driveY = 0;
		float driveZ = 0;
		float driveGyro = 0;
		bool liftLastState = false;
		bool liftState = false; //button pressed
		double liftLastTime = 0;
		double liftTime = 0;
		bool liftRan = true;
		Timer switchTimer;
		Timer grabTimer;
		switchTimer.Start();
		grabTimer.Start();


		while (IsOperatorControl() && IsEnabled())
		{
			// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
			// This sample does not use field-oriented drive, so the gyro input is set to zero.

			toSend[0] = 1;
			numToSend = 1;


			driveX = driveStick.GetRawAxis(Constants::driveXAxis);//starts driving code
			driveY = driveStick.GetRawAxis(Constants::driveYAxis);
			driveZ = driveStick.GetRawAxis(Constants::driveZAxis);
			driveGyro = gyro.GetAngle() + Constants::driveGyroTeleopOffset;


			if (driveStick.GetRawButton(Constants::driveOneAxisButton)) {//if X is greater than Y and Z, then it will only go in the direction of X
				toSend[0] = 6;
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
				toSend[0] = 7;
				driveZ = 0;//Stops Z while Z lock is pressed
			}

			if (!driveStick.GetRawButton(Constants::driveFieldLockButton)) {//robot moves based on the orientation of the field
				driveGyro = 0;//gyro stops while field lock is enabled
			}

			driveX = Constants::scaleJoysticks(driveX, Constants::driveXDeadZone, Constants::driveXMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveXDegree);
			driveY = Constants::scaleJoysticks(driveY, Constants::driveYDeadZone, Constants::driveYMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveYDegree);
			driveZ = Constants::scaleJoysticks(driveZ, Constants::driveZDeadZone, Constants::driveZMax * (.5 - (driveStick.GetRawAxis(Constants::driveThrottleAxis) / 2)), Constants::driveZDegree);
			robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ, driveGyro);//makes the robot drive




			if (pdp.GetCurrent(Constants::grabPdpChannel) < Constants::grabManualCurrent) {
				pickup.setGrabber(Constants::scaleJoysticks(grabStick.GetX(), Constants::grabDeadZone, Constants::grabMax, Constants::grabDegree)); //defines the grabber
				if(grabTimer.Get() < 1) {
					toSend[0] = 6;
				}
			}
			else {
				pickup.setGrabber(0);
				grabTimer.Reset();
				toSend[0] = 6;
			}

			if (Constants::grabLiftInverted) {
				pickup.setLifter(-Constants::scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree)); //defines the lifter
			}
			else {
				pickup.setLifter(Constants::scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree)); //defines the lifter
			}


			SmartDashboard::PutNumber("Lift Power", Constants::scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree));
			SmartDashboard::PutBoolean("Is Lifting", isLifting);

			if (Constants::scaleJoysticks(grabStick.GetY(), Constants::liftDeadZone, Constants::liftMax, Constants::liftDegree) != 0 || isLifting) { //if the robot is lifting
				isBraking = false; //stop braking thread
				SmartDashboard::PutBoolean("Braking", false);
			}
			else if(!isBraking) {
				isBraking = true; //run braking thread
				pickup.lifterBrake(isBraking);//brake the pickup
			}



			if (grabStick.GetRawButton(Constants::liftFloorButton)) {
				liftHeight = 0;
				pickup.lifterPosition(liftHeight, isLifting, grabStick);//start lifting thread
				liftRan = true;
			}

			liftTime = timer.Get();
			liftState = grabStick.GetRawButton(Constants::liftButton);

			if (liftState) { //if button is pressed
				if (!liftLastState) {
					if (liftTime - liftLastTime < Constants::liftMaxTime) {
						if (liftHeightBoxes < Constants::liftMaxHeightBoxes) {
							liftHeightBoxes++; //adds 1 to liftHeightBoxes
						}
					}
					else {
						liftHeightBoxes = 1;
						liftRamp = 0;
						liftStep = 0;
					}
				}
				liftLastTime = liftTime;
				liftLastState = true;
				liftRan = false;
			}
			else if (grabStick.GetRawButton(Constants::liftRampButton)) {
				if (liftTime - liftLastTime > Constants::liftMaxTime) {
					liftHeight = 0;
					liftStep = 0;
				}
				liftRamp = 1; //prepares to go up ramp
				liftLastTime = liftTime;
				liftRan = false;
			}
			else if (grabStick.GetRawButton(Constants::liftStepButton)) {
				if (liftTime - liftLastTime > Constants::liftMaxTime) {
					liftHeight = 0;
					liftRamp = 0;
				}
				liftStep = 1; //prepares robot for step
				liftLastTime = liftTime;
				liftRan = false;
			}
			else {
				if (liftTime - liftLastTime > Constants::liftMaxTime && !liftRan) {

					liftHeight = liftHeightBoxes * Constants::liftBoxHeight + liftRamp * Constants::liftRampHeight + liftStep * Constants::liftStepHeight; //sets liftHeight
					if (liftHeightBoxes > 0) {
						liftHeight -= Constants::liftBoxLip;
					}
					pickup.lifterPosition(liftHeight, isLifting, grabStick);//start lifting thread
					liftRan = true;
				}
				liftLastState = false;
			}

			if (grabStick.GetRawButton(Constants::grabToteButton)) {//if grab button is pressed
				grabPower = Constants::grabToteCurrent;
				backOut = true;
				if (!isGrabbing) {
					pickup.grabberGrab(isGrabbing, grabPower, backOut, grabStick);//start grabber thread
				}
			}
			else if (grabStick.GetRawButton(Constants::grabBinButton)) {//if grab button is pressed
				grabPower = Constants::grabBinCurrent;

				backOut = false;
				if (!isGrabbing) {
					pickup.grabberGrab(isGrabbing, grabPower, backOut, grabStick);//start grabber thread
				}
			}
			else if (grabStick.GetRawButton(Constants::grabChuteButton)) {//if grab button is presset
				SmartDashboard::PutBoolean("Breakpoint -2", false);
				SmartDashboard::PutBoolean("Breakpoint -1", false);
				SmartDashboard::PutBoolean("Breakpoint 0", false);
				SmartDashboard::PutBoolean("Breakpoint 1", false);
				SmartDashboard::PutBoolean("Breakpoint 2", false);
				SmartDashboard::PutBoolean("Breakpoint 3", false);
				SmartDashboard::PutBoolean("Breakpoint 4", false);
				//Wait(.5);
				if (!isGrabbing) {
					//pickup.grabberChute(isGrabbing, grabStick);//start grabber thread
				}
			}

			//determines what the LED's look like based on what the Robot is doing
			if (isGrabbing) {
				toSend[0] = 5;
				numToSend = 1;
			}
			if (isLifting) {//if the grabbing thread is running
				if (Constants::encoderToDistance(liftEncoder.Get(),Constants::liftEncoderTicks, Constants::liftEncoderBase, Constants::liftEncoderRadius) < liftHeight) {
					toSend[0] = 3;
				}
				else {
					toSend[0] = 4;
				}
				numToSend = 1;//sends 1 byte to I2C
			}

			if(!grabOuterLimit.Get()) { //tells if outer limit is hit with lights
				if(switchTimer.Get() < 1) {
					toSend[0] = 6;
				}
			}
			else {
				switchTimer.Reset();
			}

			if (driveStick.GetRawButton(Constants::sneakyMoveButton)) {
				toSend[0] = 0;
				numToSend = 1;
			}

			float distance = prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12;	// distance from ultrasonic sensor
			float rotations = (float) liftEncoder.Get();	// rotations on encoder
			SmartDashboard::PutNumber("Distance", distance);	// write stuff to smart dash
			SmartDashboard::PutNumber("Current", pdp.GetCurrent(Constants::grabPdpChannel));
			SmartDashboard::PutNumber("LED Current", pdp.GetCurrent(Constants::ledPdpChannel));
			SmartDashboard::PutNumber("Lift Encoder", rotations);
			SmartDashboard::PutNumber("Lift Height", liftHeight);
			SmartDashboard::PutNumber("Grab Encoder", grabEncoder.Get());
			SmartDashboard::PutBoolean("Grab Inner", grabInnerLimit.Get());
			SmartDashboard::PutBoolean("Grab Outer", grabOuterLimit.Get());
			SmartDashboard::PutNumber("Drive Front Left Current", pdp.GetCurrent(Constants::driveFrontLeftPin));
			SmartDashboard::PutNumber("Drive Front Right Current", pdp.GetCurrent(Constants::driveFrontRightPin));
			SmartDashboard::PutNumber("Drive Rear Left Current", pdp.GetCurrent(Constants::driveRearLeftPin));
			SmartDashboard::PutNumber("Drive Rear Right Current", pdp.GetCurrent(Constants::driveRearRightPin));
			SmartDashboard::PutNumber("Throttle", grabStick.GetZ());


			i2c.Transaction(toSend, 1, toReceive, 0);//send and receive information from arduino over I2C
			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		} //end of teleop
		isBraking = false;
		toSend[0] = 0;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);
	}

	void Autonomous()
	{
		Timer timer;
		float power = 0;
		bool isLifting = false;
		bool isGrabbing = false;
		double liftHeight = Constants::liftBoxHeight-Constants::liftBoxLip;
		double grabPower = Constants::grabAutoCurrent;
		bool backOut;

		uint8_t toSend[1];//array of bytes to send over I2C
		uint8_t toReceive[0];//array of bytes to receive over I2C
		uint8_t numToSend = 1;//number of bytes to send
		uint8_t numToReceive = 0;//number of bytes to receive
		toSend[0] = 2;//set the byte to send to 1
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);//send over I2C

		bool isSettingUp = true;

		//pickup.setGrabber(-1); //open grabber all the way
		pickup.setLifter(0.8);

		while (isSettingUp && IsEnabled() && IsAutonomous()) {
			isSettingUp = false;
			/*if (grabOuterLimit.Get() == false) {
				pickup.setGrabber(0); //open until limit
			}
			else {
				isSettingUp = true;
			}*/

			if (liftLowerLimit.Get()) {
				pickup.setLifter(0); //down till bottom
			}
			else {
				isSettingUp = true;
			}
		}

		gyro.Reset();
		liftEncoder.Reset();
		grabEncoder.Reset();

		if (grabStick.GetZ() > .8) {
			timer.Reset();
			timer.Start();
			while (timer.Get() < 1) {
				robotDrive.MecanumDrive_Cartesian(0, power, 0, gyro.GetAngle());	// drive back
				if(power>-.4){
					power-=0.005;
					Wait(.005);
				}
			}
			robotDrive.MecanumDrive_Cartesian(0, 0, 0, gyro.GetAngle());	// STOP!!!
			timer.Stop();
			timer.Reset();
			Wait(1);
		}
		power = 0;

		while (isLifting && IsEnabled() && IsAutonomous()) {
			Wait(.005);
		}

		backOut = Constants::autoBackOut;
		pickup.grabberGrab(isGrabbing, grabPower, backOut, grabStick);
		Wait(.005);

		while (isGrabbing && IsEnabled() && IsAutonomous()) {
			Wait(.005);
		}

		liftHeight = 3*Constants::liftBoxHeight;
		Wait(.005);
		pickup.lifterPosition(liftHeight, isLifting, grabStick);
		Wait(.005);
		while (isLifting && IsEnabled() && IsAutonomous()) {
			Wait(.005);
		}

		while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12 < 2 && IsEnabled() && IsAutonomous());	// while the nearest object is closer than 2 feet

		timer.Start();

		while(prox.GetVoltage() * Constants::ultrasonicVoltageToInches  < Constants::autoBackupDistance && timer.Get() < Constants::autoMaxDriveTime && IsEnabled() && IsAutonomous()) {	// while the nearest object is further than 12 feet
			if (power < .45) { //ramp up the power slowly
				power += .00375;
			}
			robotDrive.MecanumDrive_Cartesian(0, power, 0, gyro.GetAngle());	// drive back
			float distance = prox.GetVoltage() * Constants::ultrasonicVoltageToInches / 12;	// distance from ultrasonic sensor
			SmartDashboard::PutNumber("Distance", distance);	// write stuff to smart dash
			SmartDashboard::PutNumber("Drive Front Left Current", pdp.GetCurrent(Constants::driveFrontLeftPin));
			SmartDashboard::PutNumber("Drive Front Right Current", pdp.GetCurrent(Constants::driveFrontRightPin));
			SmartDashboard::PutNumber("Drive Rear Left Current", pdp.GetCurrent(Constants::driveRearLeftPin));
			SmartDashboard::PutNumber("Drive Rear Right Current", pdp.GetCurrent(Constants::driveRearRightPin));
			SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
			SmartDashboard::PutNumber("Distance (in)", prox.GetVoltage() * Constants:: ultrasonicVoltageToInches);

			Wait(.005);
		}

		timer.Reset();

		while(timer.Get() < Constants::autoBrakeTime && IsEnabled() && IsAutonomous()) {	// while the nearest object is further than 12 feet
			robotDrive.MecanumDrive_Cartesian(0,Constants::autoBrakePower,0); ///Brake
		}

		robotDrive.MecanumDrive_Cartesian(0,0,0); ///STOP!!!

		timer.Stop();
		toSend[0] = 8;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);

		while(IsAutonomous() && IsEnabled());

		toSend[0] = 0;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);
	}

	void Test() {
		robotDrive.SetSafetyEnabled(false);
		uint8_t toSend[10];//array of bytes to send over I2C
		uint8_t toReceive[10];//array of bytes to receive over I2C
		uint8_t numToSend = 1;//number of bytes to send
		uint8_t numToReceive = 0;//number of bytes to receive
		toSend[0] = 7; //send 0 to arduino
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);

		bool isSettingUp = true;

		pickup.setGrabber(-1);
		pickup.setLifter(1);
		while (isSettingUp) {
			isSettingUp = false;
			if (grabOuterLimit.Get() == false) {
				pickup.setGrabber(0);
			}
			else {
				isSettingUp = true;
			}

			if (liftLowerLimit.Get()) {
				pickup.setLifter(0);
			}
			else {
				isSettingUp = true;
			}
		}
		gyro.Reset();
		liftEncoder.Reset();
		grabEncoder.Reset();

		toSend[0] = 8;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);

		while(IsTest() && IsEnabled());

		toSend[0] = 0;
		i2c.Transaction(toSend, numToSend, toReceive, numToReceive);
	}

};

START_ROBOT_CLASS(Robot);
