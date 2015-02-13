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

inline void lifterPositionTaskFunc(uint32_t joystickPtr, uint32_t liftTalon1Ptr, uint32_t liftTalon2Ptr, uint32_t liftEncoderPtr, uint32_t liftUpperLimitPtr, uint32_t liftLowerLimitPtr, uint32_t pdpPtr, uint32_t heightPtr, uint32_t isLiftingPtr ...) {//uint is a pointer and not an integer
	double *height = (double *) heightPtr;//initializes double
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *liftTalon1 = (Talon *) liftTalon1Ptr;
	Talon *liftTalon2 = (Talon *) liftTalon2Ptr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftLowerLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;
	bool *isLifting = (bool *) isLiftingPtr;

	*isLifting = true;//tells robot.cpp that thread is running

	if (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, Constants::liftEncoderBase, Constants::liftEncoderRadius) > *height) {//checks to see if encoder is higher or lower than it's supposed to be
		if (liftLowerLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon1->Set(1);//move down
				liftTalon2->Set(1);//move down
				Wait(Constants::liftDelay);
		}
		while (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, Constants::liftEncoderBase, Constants::liftEncoderRadius) < *height && pdp->GetCurrent(Constants::lift1PdpChannel) < Constants::liftCurrent && pdp->GetCurrent(Constants::lift2PdpChannel) < Constants::liftCurrent && liftLowerLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too high and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get()/Constants::liftEncoderTicks);//displays number of ticks of encoder in SmartDashboard
			liftTalon1->Set(1);//move down
			liftTalon2->Set(1);//move down
		}
	}
	else {
		if (liftUpperLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon1->Set(-1);//move up
				liftTalon2->Set(-1);//move up
				Wait(Constants::liftDelay);
		}
		while (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, Constants::liftEncoderBase, Constants::liftEncoderRadius) < *height && pdp->GetCurrent(Constants::lift1PdpChannel) < Constants::liftCurrent && pdp->GetCurrent(Constants::lift2PdpChannel) < Constants::liftCurrent && liftUpperLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too low and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Encoder",liftEncoder->Get()/Constants::liftEncoderTicks);//displays number of ticks of encoder on SmartDashboard
			liftTalon1->Set(-1);//move up
			liftTalon2->Set(-1);//move up
		}
	}

	liftTalon1->Set(0);//stop
	liftTalon2->Set(0);//stop
	*isLifting = false;//tells robot.cpp that thread is finished
}


Pickup::Pickup(Talon &grabTalonPtr, Switch &grabInnerLimitPtr, Switch &grabOuterLimitPtr, Talon &liftTalon1Ptr, Talon &liftTalon2Ptr, Encoder &liftEncoderPtr, Switch &liftUpperLimitPtr, Switch &liftLowerLimitPtr, PowerDistributionPanel &pdpPtr)://initalizes objects
		grabTalon(grabTalonPtr),//initializes objects from pointers
		grabInnerLimit(grabInnerLimitPtr),
		grabOuterLimit(grabOuterLimitPtr),
		grabberPositionTask("grabberPosition", (FUNCPTR) grabberPositionTaskFunc),//creates task from inline functions above
		liftTalon1(liftTalon1Ptr),
		liftTalon2(liftTalon2Ptr),
		liftEncoder(liftEncoderPtr),
		liftUpperLimit(liftUpperLimitPtr),
		liftLowerLimit(liftLowerLimitPtr),
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
	grabberPositionTask.Start((uint32_t) &joystick, (uint32_t) &grabTalon, (uint32_t) &grabInnerLimit, (uint32_t) &pdp, (uint32_t) &isGrabbing);//casts all pointers to uint 32 so they can run as a thread
}

void Pickup::setLifter(float power)//moves lifter and checks limit switches
{
	if (power > 0) {	// if it wants to move down
		if (liftLowerLimit.Get() == false) {	// if it is not already all the way down
			liftTalon1.Set(power);	// move out
			liftTalon2.Set(power);	// move out
		}
		else {
			liftTalon1.Set(0);
			liftTalon2.Set(0);
		}
	}
	else {	// if it wants to move in
		if (liftUpperLimit.Get() == false) {	//if it is not already all the way in
			liftTalon1.Set(power);	// move in
			liftTalon2.Set(power);	// move in
		}
		else {
			liftTalon1.Set(0);
			liftTalon2.Set(0);
		}
	}
}

void Pickup::lifterPosition(double &height, bool &isLifting, Joystick &joystick) {//starts lifting thread
	lifterPositionTask.Start((uint32_t) &joystick, (uint32_t) &liftTalon1,  (uint32_t) &liftTalon2, (uint32_t) &liftEncoder, (uint32_t) &liftUpperLimit, (uint32_t) &liftLowerLimit, (uint32_t) &pdp, (uint32_t) &height, (uint32_t) &isLifting);//casts all pointers to uint 32 so they can run as a thread
}

