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

inline void grabberPositionTaskFunc(uint32_t joystickPtr, uint32_t grabTalonPtr, uint32_t grabInnerLimitPtr, uint32_t pdpPtr, uint32_t isGrabbingPtr ...) {//uint is a pointer and not an integer
	Joystick *joystick = (Joystick *) joystickPtr;//initializes objects from pointers
	Talon *grabTalon = (Talon *) grabTalonPtr;
	Switch *grabInnerLimit = (Switch *) grabInnerLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;
	bool *isGrabbing = (bool *) isGrabbingPtr;

	*isGrabbing = true;//tells robot.cpp that thread is running

	if (grabInnerLimit->Get() == false) {//starts to spin motor to pass startup current
		grabTalon->Set(1);//move in
		Wait(Constants::grabDelay);
	}


	while (pdp->GetCurrent(Constants::grabPdpChannel) < Constants::grabCurrent && grabInnerLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it hasn't reached the current cutoff, hit a limit switch, or been cancelled
		grabTalon->Set(1);
		SmartDashboard::PutNumber("Current",pdp->GetCurrent(Constants::grabPdpChannel));//displays current on SmartDashboard
	}

	grabTalon->Set(0);//stop moving
	*isGrabbing = false;//tells that thread is over
}

inline void lifterPositionTaskFunc(uint32_t joystickPtr, uint32_t liftTalonPtr, uint32_t liftEncoderPtr, uint32_t liftLowerLimitPtr, uint32_t liftUpperLimitPtr, uint32_t pdpPtr, uint32_t heightPtr, uint32_t isLiftingPtr ...) {//uint is a pointer and not an integer
	double *height = (double *) heightPtr;//initializes double
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *liftTalon = (Talon *) liftTalonPtr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftLowerLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;
	bool *isLifting = (bool *) isLiftingPtr;

	*isLifting = true;//tells robot.cpp that thread is running

	if (liftEncoder->Get() > *height * 20) {//checks to see if encoder is higher or lower than it's supposed to be
		if (liftUpperLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon->Set(1);//move in
				Wait(Constants::liftDelay);
		}
		while (liftEncoder->Get() > *height * 200 && pdp->GetCurrent(Constants::liftPdpChannel) < Constants::liftCurrent && liftUpperLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too high and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get());//displays number of ticks of encoder in SmartDashboard
			liftTalon->Set(1);//move down
		}
	}
	else {
		if (liftLowerLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon->Set(-1);//move in
				Wait(Constants::liftDelay);
		}
		while (liftEncoder->Get() < *height * 200 && pdp->GetCurrent(Constants::liftPdpChannel) < Constants::liftCurrent && liftLowerLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too low and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get());//displays number of ticks of encoder on SmartDashboard
			liftTalon->Set(-1);//move up
		}
	}

	liftTalon->Set(0);//stop
	*isLifting = false;//tells robot.cpp that thread is finished
}


Pickup::Pickup(Talon &grabTalonPtr, Switch &grabInnerLimitPtr, Switch &grabOuterLimitPtr, Talon &liftTalonPtr, Encoder &liftEncoderPtr, Switch &liftInnerLimitPtr, Switch &liftOuterLimitPtr, PowerDistributionPanel &pdpPtr)://initalizes objects
		grabTalon(grabTalonPtr),//initializes objects from pointers
		grabInnerLimit(grabInnerLimitPtr),
		grabOuterLimit(grabOuterLimitPtr),
		grabberPositionTask("grabberPosition", (FUNCPTR) grabberPositionTaskFunc),//creates task from inline functions above
		liftTalon(liftTalonPtr),
		liftEncoder(liftEncoderPtr),
		liftInnerLimit(liftInnerLimitPtr),
		liftOuterLimit(liftOuterLimitPtr),
		lifterPositionTask("lifterPosition", (FUNCPTR) lifterPositionTaskFunc),
		pdp(pdpPtr)
{
}
void Pickup::setGrabber(float power)//moves Grabber and checks limit switches
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

void Pickup::grabberPosition(bool &isGrabbing, Joystick &joystick) {//start grabber thread
	grabberPositionTask.Start((uint32_t) &joystick, (uint32_t) &grabTalon, (uint32_t) &grabInnerLimit, (uint32_t) &pdp, (uint32_t) &pdp, (uint32_t) &isGrabbing);//casts all pointers to uint 32 so they can run as a thread
}

void Pickup::setLifter(float power)//moves lifter and checks limit switches
{
	if (power > 0) {	// if it wants to move out
		if (liftOuterLimit.Get() == false) {	// if it is not already all the way out
			liftTalon.Set(power);	// move out
		}
		else {
			liftTalon.Set(0);
		}
	}
	else {	// if it wants to move in
		if (liftInnerLimit.Get() == false) {	//if it is not already all the way in
			liftTalon.Set(power);	// move in
		}
		else {
			liftTalon.Set(0);
		}
	}
}

void Pickup::lifterPosition(double &height, bool &isLifting, Joystick &joystick) {//starts lifting thread
	lifterPositionTask.Start((uint32_t) &joystick, (uint32_t) &liftTalon, (uint32_t) &liftEncoder, (uint32_t) &liftInnerLimit, (uint32_t) &liftOuterLimit, (uint32_t) &pdp, (uint32_t) &height, (uint32_t) &isLifting);//casts all pointers to uint 32 so they can run as a thread
}

