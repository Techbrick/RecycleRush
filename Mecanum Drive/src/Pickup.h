/*
 * Pickup.h
 *
 *  Created on: Jan 12, 2015
 *      Author: Mitchell Roberts
 */

#ifndef PICKUP_H_
#define PICKUP_H_

#include "pthread.h"
#include "WPILib.h"
#include "Switch.h"

class Pickup {
	Talon &grabTalon;
	Switch &grabInnerLimit;
	Switch &grabOuterLimit;
	Task grabberPositionTask;
	Talon &liftTalon;
	Encoder &liftEncoder;
	Switch &liftUpperLimit;
	Switch &liftLowerLimit;
	Task lifterPositionTask;
	Task lifterBrakeTask;
	PowerDistributionPanel &pdp;
public:
	Pickup(Talon &grabTalonPtr, Switch &grabInnerLimitPtr, Switch &grabOuterLimitPtr, Talon &liftTalonPtr, Encoder &liftEncoderPtr, Switch &liftInnerLimitPtr, Switch &liftOuterLimitPtr, PowerDistributionPanel &pdpPtr);
	void setGrabber(float power);	// opens and closes grabber
	void grabberGrab(bool &isGrabbing, Joystick &joystick);	// move grabber to set position (threaded)
	void setLifter(float power);	// moves forklift up and down
	void lifterPosition(double &height, bool &isGrabbing, Joystick &joystick, bool &changebase);	// move forklift to set position (threaded)
	void lifterBrake(bool &isBraking);	// hold forklift to set position (threaded)

};

#endif /* PICKUP_H_ */
