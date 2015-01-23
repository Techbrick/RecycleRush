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

inline void grabberPositionTaskFunc(uint32_t joystickPtr, uint32_t grabTalonPtr, uint32_t grabInnerLimitPtr, uint32_t pdpPtr, uint32_t isGrabbingPtr ...) {
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *grabTalon = (Talon *) grabTalonPtr;
	Switch *grabInnerLimit = (Switch *) grabInnerLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;
	bool isGrabbing = *((bool *) isGrabbingPtr);

	isGrabbing = true;

	while (pdp->GetCurrent(Constants::grabPdpChannel) < Constants::grabCurrent && grabInnerLimit->Get() == false && joystick->GetRawButton(Constants::cancelButton) == false) {
		grabTalon->Set(1);
		SmartDashboard::PutNumber("Current",pdp->GetCurrent(Constants::grabPdpChannel));
	}

	grabTalon->Set(0);
	isGrabbing = false;
}

inline void lifterPositionTaskFunc(uint32_t joystickPtr, uint32_t liftTalonPtr, uint32_t liftEncoderPtr, uint32_t liftLowerLimitPtr, uint32_t liftUpperLimitPtr, uint32_t heightPtr, uint32_t isLiftingPtr ...) {
	double height = *((double *) heightPtr);
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *liftTalon = (Talon *) liftTalonPtr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftLowerLimitPtr;
	bool isLifting = *((bool *) isLiftingPtr);

	isLifting = true;

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
	isLifting = false;
}


Pickup::Pickup(Talon &grabTalonPtr, Switch &grabInnerLimitPtr, Switch &grabOuterLimitPtr, Talon &liftTalonPtr, Encoder &liftEncoderPtr, Switch &liftInnerLimitPtr, Switch &liftOuterLimitPtr, PowerDistributionPanel &pdpPtr):
		grabTalon(grabTalonPtr),
		grabInnerLimit(grabInnerLimitPtr),
		grabOuterLimit(grabOuterLimitPtr),
		grabberPositionTask("grabberPosition", (FUNCPTR) grabberPositionTaskFunc),
		liftTalon(liftTalonPtr),
		liftEncoder(liftEncoderPtr),
		liftInnerLimit(liftInnerLimitPtr),
		liftOuterLimit(liftOuterLimitPtr),
		lifterPositionTask("lifterPosition", (FUNCPTR) lifterPositionTaskFunc),
		pdp(pdpPtr)
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

void Pickup::grabberPosition(bool &isGrabbing, Joystick &joystick) {
	grabberPositionTask.Start((uint32_t) &joystick, (uint32_t) &grabTalon, (uint32_t) &grabInnerLimit, (uint32_t) &pdp, (uint32_t) &isGrabbing);
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

void Pickup::lifterPosition(double &height, bool &isLifting, Joystick &joystick) {
	lifterPositionTask.Start((uint32_t) &joystick, (uint32_t) &liftTalon, (uint32_t) &liftEncoder, (uint32_t) &liftInnerLimit, (uint32_t) &liftOuterLimit, (uint32_t) &height, (uint32_t) &isLifting);
}

