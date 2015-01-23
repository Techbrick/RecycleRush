/*
 * Pickup.cpp
 *
 *  Created on: Jan 12, 2015
 *      Author: Mitchell Roberts
 */

#include "Pickup.h"
#include "WPILib.h"
#include "pthread.h"
#include "Constants.cpp"
#include "Switch.h"

inline void grabberPositionTaskFunc(uint32_t joystickPtr, uint32_t grabTalonPtr, uint32_t grabInnerLimitPtr, uint32_t pdpPtr, ...) {
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *grabTalon = (Talon *) grabTalonPtr;
	Switch *grabInnerLimit = (Switch *) grabInnerLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;

	/*for (int i = 1; i < 1000; i++) {
		grabTalon.Set(i/1000.0);
		Wait(1.0/2000);
	}*/

	while (pdp->GetCurrent(Constants::grabPdpChannel) < Constants::grabCurrent && grabInnerLimit->Get() == false && joystick->GetRawButton(Constants::cancelButton) == false) {
		grabTalon->Set(1);
		SmartDashboard::PutNumber("Current",pdp->GetCurrent(Constants::grabPdpChannel));
	}

	grabTalon->Set(0);
}

inline void lifterPositionTaskFunc(uint32_t joystickPtr, uint32_t liftTalonPtr, uint32_t liftEncoderPtr, uint32_t liftLowerLimitPtr, uint32_t liftUpperLimitPtr, uint32_t heightPtr, ...) {
	double height = *((double *) heightPtr);
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *liftTalon = (Talon *) liftTalonPtr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftLowerLimitPtr;

	if (liftEncoder->Get() > height * 20) {
		while (liftEncoder->Get() > height * 200 && liftUpperLimit->Get() == false && joystick->GetRawButton(Constants::cancelButton) == false) {
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get());
			liftTalon->Set(1);
		}
	}
	else {
		while (liftEncoder->Get() < height * 200 && liftLowerLimit->Get() == false && joystick->GetRawButton(Constants::cancelButton) == false) {
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get());
			liftTalon->Set(-1);
		}
	}

	liftTalon->Set(0);
}


Pickup::Pickup(Talon &grabTalonPtr):
		grabTalon(grabTalonPtr),
		grabInnerLimit(Constants::grabInnerLimitPin),
		grabOuterLimit(Constants::grabOuterLimitPin),
		grabberPositionTask("grabberPosition", (FUNCPTR) grabberPositionTaskFunc),
		liftTalon(Constants::liftTalonPin),
		liftEncoder(Constants::liftEncoderAPin, Constants::liftEncoderBPin),
		liftInnerLimit(Constants::liftInnerLimitPin),
		liftOuterLimit(Constants::liftOuterLimitPin),
		lifterPositionTask("lifterPosition", (FUNCPTR) lifterPositionTaskFunc),
		pdp()
{
}

void Pickup::setGrabber(float power)
{
	if (power > 0) {	// if it wants to move out
		if (grabOuterLimit.Get() == false) {	// if it is not already all the way out
			grabTalon.Set(power);	// move out
		}
		else {
			grabTalon.Set(0);
		}
	}
	else {	// if it wants to move in
		if (grabInnerLimit.Get() == false) {	//if it is not already all the way in
			grabTalon.Set(power);	// move in
		}
		else {
			grabTalon.Set(0);
		}
	}
}

void Pickup::grabberPosition(double &width, Joystick &joystick) {
	grabberPositionTask.Start((uint32_t) &joystick, (uint32_t) &grabTalon, (uint32_t) &grabInnerLimit, (uint32_t) &pdp);
}

void Pickup::setLifter(float power)
{
	if (power > 0) {	// if it wants to move out
		if (grabOuterLimit.Get() == false) {	// if it is not already all the way out
			grabTalon.Set(power);	// move out
		}
		else {
			grabTalon.Set(0);
		}
	}
	else {	// if it wants to move in
		if (grabInnerLimit.Get() == false) {	//if it is not already all the way in
			grabTalon.Set(power);	// move in
		}
		else {
			grabTalon.Set(0);
		}
	}
}

void Pickup::lifterPosition(double &height, Joystick &joystick) {
	lifterPositionTask.Start((uint32_t) &joystick, (uint32_t) &liftTalon, (uint32_t) &liftEncoder, (uint32_t) &liftInnerLimit, (uint32_t) &liftOuterLimit, (uint32_t) &height);
}

