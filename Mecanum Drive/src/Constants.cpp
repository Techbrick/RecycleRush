/*
 * Constants.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Mitchell Roberts
 */

class Constants {
public:
	const static constexpr int frontLeftPin	= 2;
	const static constexpr int rearLeftPin	= 3;
	const static constexpr int frontRightPin = 1;
	const static constexpr int rearRightPin	= 0;

	const static constexpr int driveStickChannel = 0;
	const static constexpr int grabStickChannel	= 1;

	const static constexpr int grabCurrent = 20;
	const static constexpr int grabPdpChannel = 14;

	const static constexpr int ultrasonicPin = 0;
	const static constexpr float ultrasonicVoltageToInches = 512.0/5;

	const static constexpr int grabTalonPin = 4;
	const static constexpr int grabInnerLimitPin = 2;
	const static constexpr int grabOuterLimitPin = 3;

	const static constexpr int liftEncoderAPin = 4;
	const static constexpr int liftEncoderBPin = 5;
	const static constexpr int liftEncoderTicks = 360;

	const static constexpr int liftTalonPin = 5;
	const static constexpr int liftInnerLimitPin = 6;
	const static constexpr int liftOuterLimitPin = 7;

	const static constexpr float xDeadZone = .2;
	const static constexpr float xMax = 1;
	const static constexpr int xDegree = 3;
	const static constexpr float yDeadZone = .2;
	const static constexpr float yMax = 1;
	const static constexpr int yDegree = 3;
	const static constexpr float zDeadZone = .2;
	const static constexpr float zMax = .5;
	const static constexpr int zDegree = 3;
	const static constexpr float grabDeadZone = .2;
	const static constexpr float grabMax = 1;
	const static constexpr int grabDegree = 3;
	const static constexpr float liftDeadZone = .2;
	const static constexpr float liftMax = 1;
	const static constexpr int liftDegree = 3;


	const static constexpr int cancelButton = 3;

	Constants() {}

};
