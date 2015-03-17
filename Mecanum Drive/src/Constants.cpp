/*
 * Constants.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Mitchell Roberts
 */

#include <math.h>

class Constants {
public:

	//PWM Out Pins
	const static constexpr int driveFrontLeftPin = 2;
	const static constexpr int driveRearLeftPin	= 3;
	const static constexpr int driveFrontRightPin = 1;
	const static constexpr int driveRearRightPin = 0;
	const static constexpr int grabTalonPin = 5;
	const static constexpr int liftTalonPin = 6;


	//Digital I/O Pins
	const static constexpr int liftEncoderAPin = 0;
	const static constexpr int liftEncoderBPin = 1;
	const static constexpr int liftUpperLimitPin = 2;
	const static constexpr int liftLowerLimitPin = 3;
	const static constexpr int grabInnerLimitPin = 4;
	const static constexpr int grabOuterLimitPin = 5;
	const static constexpr int grabEncoderAPin = 6;
	const static constexpr int grabEncoderBPin = 7;


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
	const static constexpr bool grabLiftInverted = false;
	const static constexpr int grabStickChannel	= 1;
	const static constexpr int pickupCancelButton = 2;
	const static constexpr int grabButton = 1;
	const static constexpr int liftButton = 3;
	const static constexpr int liftRampButton = 6;
	const static constexpr int liftStepButton = 7;


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
	const static constexpr int grabEncoderTicks = 2048;
	const static constexpr bool liftEncoderReverse = true;
	const static constexpr float liftEncoderRadius = 1.4722936;
	const static constexpr float liftEncoderBase = 8.875;
	const static constexpr float grabDelay = .25;
	const static constexpr int grabCurrent = 15;
	const static constexpr float liftDelay = .2;
	const static constexpr int liftCurrent = 100;
	const static constexpr float ultrasonicVoltageToInches = 512.0/5;
	const static constexpr int ledAddress = 5;
	const static constexpr double liftMaxTime = 1.0;
	const static constexpr int liftMaxHeightBoxes = 5;
	const static constexpr int liftRampHeight = 3;
	const static constexpr int liftStepHeight = 8.5;
	const static constexpr float liftBoxHeight = 12;
	const static constexpr float liftBoxLip = 3.5;
	const static constexpr bool liftBrakeIsEnabled = false;
	const static constexpr float liftBrakeP = -0.06;
	const static constexpr float liftBrakeI = -0.0;
	const static constexpr float liftBrakeD = -0.0;
	const static constexpr float liftBrakeUpPower = -0.1;
	const static constexpr float liftBrakeUpTime = 0.25;
	const static constexpr float driveGyroTeleopOffset = 180;
	const static constexpr float autoBackupDistance = 144;
	const static constexpr float autoMaxDriveTime = 4;
	const static constexpr float autoBrakeTime = .25;
	const static constexpr float autoBrakePower = -.05;

	Constants() {}

	// scale joystick input to avoid crazy driving
	static float scaleJoysticks(float power, float dead, float max, int degree) {
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

	static float encoderToDistance (int value, float ticksPerRotation, float base, float radius) {
		return value / ticksPerRotation * M_PI * 2 * radius + base;
	}

};

