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
	Timer timer;
	timer.Start();

	*isGrabbing = true;//tells robot.cpp that thread is running

	while (grabInnerLimit->Get() && timer.Get() < Constants::grabDelay) {//starts to spin motor to pass startup current
		grabTalon->Set(1);//move in
	}

	timer.Stop();

	while (pdp->GetCurrent(Constants::grabPdpChannel) < Constants::grabCurrent && grabInnerLimit->Get() && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it hasn't reached the current cutoff, hit a limit switch, or been cancelled
		grabTalon->Set(1);
		SmartDashboard::PutNumber("Current",pdp->GetCurrent(Constants::grabPdpChannel));//displays current on SmartDashboard
	}

	grabTalon->Set(0);//stop moving
	*isGrabbing = false;//tells that thread is over
}

inline void lifterPositionTaskFunc(uint32_t joystickPtr, uint32_t liftTalonPtr, uint32_t liftEncoderPtr, uint32_t liftUpperLimitPtr, uint32_t liftLowerLimitPtr, uint32_t pdpPtr, uint32_t heightPtr, uint32_t isLiftingPtr, uint32_t changebasePtr ...) {//uint is a pointer and not an integer
	double *height = (double *) heightPtr;//initializes double
	Joystick *joystick = (Joystick *) joystickPtr;
	Talon *liftTalon = (Talon *) liftTalonPtr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftUpperLimitPtr;
	PowerDistributionPanel *pdp = (PowerDistributionPanel *) pdpPtr;
	bool *isLifting = (bool *) isLiftingPtr;
	bool *changebase= (bool *) changebasePtr;
	float base;
	*isLifting = true;//tells robot.cpp that thread is running
	if(changebase){
		base=Constants::updatedLiftEncoderBase;
	}
	else{
		base=Constants::liftEncoderBase;
	}
	if (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, base, Constants::liftEncoderRadius) > *height) {//checks to see if encoder is higher than it's supposed to be
		if (liftLowerLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon->Set(1);//move down
				Wait(Constants::liftDelay);
		}
		while (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, base, Constants::liftEncoderRadius) > *height && pdp->GetCurrent(Constants::liftPdpChannel) < Constants::liftCurrent && liftLowerLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too high and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Pretend Encoder",liftEncoder->Get());//displays number of ticks of encoder in SmartDashboard
			liftTalon->Set(.7);//move down
		}
	}
	else {
		if (liftUpperLimit->Get() == false) {//starts to spin motor to pass startup current
				liftTalon->Set(-1);//move up
				Wait(Constants::liftDelay);
		}
		while (Constants::encoderToDistance(liftEncoder->Get(),Constants::liftEncoderTicks, base, Constants::liftEncoderRadius) < *height && pdp->GetCurrent(Constants::liftPdpChannel) < Constants::liftCurrent && liftUpperLimit->Get() == false && joystick->GetRawButton(Constants::pickupCancelButton) == false) {//while it is too low and hasn't hit a limit switch or been cancelled
			SmartDashboard::PutNumber("Pretend Encoder",liftEncoder->Get());//displays number of ticks of encoder on SmartDashboard
			liftTalon->Set(-1);//move up
		}
	}

	liftTalon->Set(0);//stop
	*isLifting = false;//tells robot.cpp that thread is finished
}

inline void lifterBrakeTaskFunc(uint32_t liftTalonPtr, uint32_t liftEncoderPtr, uint32_t liftUpperLimitPtr, uint32_t liftLowerLimitPtr, uint32_t isBrakingPtr ...) {//uint is a pointer and not an integer
	Talon *liftTalon = (Talon *) liftTalonPtr;
	Encoder *liftEncoder = (Encoder *) liftEncoderPtr;
	Switch *liftLowerLimit = (Switch *) liftLowerLimitPtr;
	Switch *liftUpperLimit = (Switch *) liftUpperLimitPtr;
	bool *isBraking = (bool *) isBrakingPtr;
	PIDController pid(Constants::liftBrakeP, Constants::liftBrakeI, Constants::liftBrakeD, liftEncoder, liftTalon);
	Timer timer;

	//TODO enable and test out brake
	if (!Constants::liftBrakeIsEnabled) {
		return;
	}

	timer.Start();

	while (timer.Get() < Constants::liftBrakeUpTime) {
		liftTalon->Set(Constants::liftBrakeUpPower);
	}

	liftTalon->Set(0);

	pid.Enable();
	pid.SetSetpoint(liftEncoder->Get());

	while (*isBraking) {
		if ((pid.Get() > 0 && liftLowerLimit->Get()) || (pid.Get() < 0 && liftUpperLimit->Get())){
			liftTalon->Set(0);
		}
		else {
			liftTalon->Set(-pid.Get());
		}
		SmartDashboard::PutBoolean("Braking", true);


	}

	pid.Disable();
	SmartDashboard::PutBoolean("Braking", false);
	liftTalon->Set(0);
}


Pickup::Pickup(Talon &grabTalonPtr, Switch &grabInnerLimitPtr, Switch &grabOuterLimitPtr, Talon &liftTalonPtr, Encoder &liftEncoderPtr, Switch &liftUpperLimitPtr, Switch &liftLowerLimitPtr, PowerDistributionPanel &pdpPtr)://initalizes objects
		grabTalon(grabTalonPtr),//initializes objects from pointers
		grabInnerLimit(grabInnerLimitPtr),
		grabOuterLimit(grabOuterLimitPtr),
		grabberPositionTask("grabberPosition", (FUNCPTR) grabberPositionTaskFunc),//creates task from inline functions above
		liftTalon(liftTalonPtr),
		liftEncoder(liftEncoderPtr),
		liftUpperLimit(liftUpperLimitPtr),
		liftLowerLimit(liftLowerLimitPtr),
		lifterPositionTask("lifterPosition", (FUNCPTR) lifterPositionTaskFunc),
		lifterBrakeTask("lifterBrake", (FUNCPTR) lifterBrakeTaskFunc),
		pdp(pdpPtr)
{
}
void Pickup::setGrabber(float power)//moves Grabber and checks limit switches
{
	if (power < 0) {	// if it wants to move out
		if (grabOuterLimit.Get()) {	// if it is not already all the way out
			grabTalon.Set(power);	// move out
		}
		else {
			grabTalon.Set(0);
		}
	}
	else {	// if it wants to move in
		if (grabInnerLimit.Get()) {	//if it is not already all the way in
			grabTalon.Set(power);	// move in
		}
		else {
			grabTalon.Set(0);
		}
	}
}

void Pickup::grabberGrab(bool &isGrabbing, Joystick &joystick) {//start grabber thread
	grabberPositionTask.Start((uint32_t) &joystick, (uint32_t) &grabTalon, (uint32_t) &grabInnerLimit, (uint32_t) &pdp, (uint32_t) &isGrabbing);//casts all pointers to uint 32 so they can run as a thread
}

void Pickup::setLifter(float power)//moves lifter and checks limit switches
{
	if (power > 0) {	// if it wants to move down
		if (liftLowerLimit.Get() == false) {	// if it is not already all the way down
			liftTalon.Set(power);	// move down
		}
		else {
			liftTalon.Set(0);
		}
	}
	else {	// if it wants to move up
		if (liftUpperLimit.Get() == false) {	//if it is not already all the way up
			liftTalon.Set(power);	// move up
		}
		else {
			liftTalon.Set(0);
		}
	}
}

void Pickup::lifterPosition(double &height, bool &isLifting, Joystick &joystick, bool &changebase) {//starts lifting thread
	lifterPositionTask.Start((uint32_t) &joystick, (uint32_t) &liftTalon, (uint32_t) &liftEncoder, (uint32_t) &liftUpperLimit, (uint32_t) &liftLowerLimit, (uint32_t) &pdp, (uint32_t) &height, (uint32_t) &isLifting, (uint32_t) &changebase);//casts all pointers to uint 32 so they can run as a thread
}

void Pickup::lifterBrake(bool &isBraking) {//starts lifting thread
	lifterBrakeTask.Start((uint32_t) &liftTalon, (uint32_t) &liftEncoder, (uint32_t) &liftUpperLimit, (uint32_t) &liftLowerLimit, (uint32_t) &isBraking); //uint is a pointer and not an integer
}
