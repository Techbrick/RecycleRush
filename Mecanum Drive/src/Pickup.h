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
	Talon &liftTalon1;
	Talon &liftTalon2;
	Encoder &liftEncoder;
	Switch &liftUpperLimit;
	Switch &liftLowerLimit;
	Task lifterPositionTask;
	PowerDistributionPanel &pdp;
public:
	Pickup(Talon &grabTalonPtr, Switch &grabInnerLimit, Switch &grabOuterLimit, Talon &liftTalon1, Talon &liftTalon2, Encoder &liftEncoder, Switch &liftInnerLimit, Switch &liftOuterLimit, PowerDistributionPanel &pdpPtr);
	void setGrabber(float power);	// opens and closes grabber
	void grabberPosition(bool &isGrabbing, Joystick &joystick);	// move grabber to set position (threaded)
	void setLifter(float power);	// moves forklift up and down
	void lifterPosition(double &height, bool &isGrabbing, Joystick &joystick);	// move forklift to set position (threaded)

};

#endif /* PICKUP_H_ */
