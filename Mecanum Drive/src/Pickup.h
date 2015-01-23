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
	Switch grabInnerLimit;
	Switch grabOuterLimit;
	Task grabberPositionTask;
	Talon liftTalon;
	Encoder liftEncoder;
	Switch liftInnerLimit;
	Switch liftOuterLimit;
	Task lifterPositionTask;
	PowerDistributionPanel pdp;
public:
	Pickup(Talon &grabTalonPtr);
	void setGrabber(float power);	// opens and closes grabber
	void grabberPosition(double &width, Joystick &joystick);	// move grabber to set position (threaded)
	void setLifter(float power);	// moves forklift up and down
	void lifterPosition(double &height, Joystick &joystick);	// move forklift to set position (threaded)

};

#endif /* PICKUP_H_ */
