/*
 * Switch.h
 *
 *  Created on: Jan 17, 2015
 *      Author: Mitchell Roberts
 */

#ifndef SRC_SWITCH_H_
#define SRC_SWITCH_H_

#include "WPILib.h"

class Switch : public DigitalInput {
public:
	Switch(uint32_t channel);
	bool Get();
};

#endif /* SRC_SWITCH_H_ */
