/*
 * Constants.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Mitchell Roberts
 */

class Constants {
public:

	//PWM Out Pins
	const static constexpr int driveFrontLeftPin = 2;
	const static constexpr int driveRearLeftPin	= 3;
	const static constexpr int driveFrontRightPin = 1;
	const static constexpr int driveRearRightPin = 0;
	const static constexpr int grabTalonPin = 5;
	const static constexpr int liftTalonPin = 4;


	//Digital I/O Pins
	const static constexpr int liftEncoderAPin = 0;
	const static constexpr int liftEncoderBPin = 1;
	const static constexpr int liftInnerLimitPin = 2;
	const static constexpr int liftOuterLimitPin = 3;
	const static constexpr int grabInnerLimitPin = 4;
	const static constexpr int grabOuterLimitPin = 5;


	//Analog In Pins
	const static constexpr int driveGyroRate = 0;
	const static constexpr int driveGyroTemp = 1;
	const static constexpr int driveUltrasonicPin = 3;


	//PDP Ports
	const static constexpr int grabPdpChannel = 15;
	const static constexpr int ledPdpChannel = 13;
	const static constexpr int liftPdpChannel = 14;


	//Drive Stick
	const static constexpr int driveStickChannel = 0;
	/*const static constexpr int driveXAxis = 0;
	const static constexpr int driveYAxis = 1;
	const static constexpr int driveZAxis = 5;
	const static constexpr int driveThrottleAxis = 2;
	const static constexpr int driveOneAxisButton = 7;
	const static constexpr int driveXYButton = 5;
	const static constexpr int driveFieldLockButton = 1;*/
	const static constexpr int driveXAxis = 0;
	const static constexpr int driveYAxis = 1;
	const static constexpr int driveZAxis = 2;
	const static constexpr int driveThrottleAxis = 3;
	const static constexpr int driveOneAxisButton = 11;
	const static constexpr int driveXYButton = 2;
	const static constexpr int driveFieldLockButton = 1;


	//Grab Stick
	const static constexpr int grabStickChannel	= 1;
	const static constexpr int pickupCancelButton = 3;
	const static constexpr int grabButton = 1;
	const static constexpr int liftButton = 2;



	//Joystick Scaling Constants
	const static constexpr float driveXDeadZone = .2;
	const static constexpr float driveXMax = 1;
	const static constexpr int driveXDegree = 1;
	const static constexpr float driveYDeadZone = .2;
	const static constexpr float driveYMax = 1;
	const static constexpr int driveYDegree = 1;
	const static constexpr float driveZDeadZone = .2;
	const static constexpr float driveZMax = .5;
	const static constexpr int driveZDegree = 1;
	const static constexpr float grabDeadZone = .2;
	const static constexpr float grabMax = 1;
	const static constexpr int grabDegree = 3;
	const static constexpr float liftDeadZone = .2;
	const static constexpr float liftMax = 1;
	const static constexpr int liftDegree = 3;

	//Sensor Constants
	const static constexpr int liftEncoderTicks = 360;
	const static constexpr float grabDelay = .07;
	const static constexpr int grabCurrent = 10;
	const static constexpr float liftDelay = .2;
	const static constexpr int liftCurrent = 20;
	const static constexpr float ultrasonicVoltageToInches = 512.0/5;
	const static constexpr int ledAddress = 5;


	Constants() {}

};

