/*
 * Switch.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: Mitchell
 */

#include <Switch.h>

Switch::Switch(uint32_t channel) :
DigitalInput(channel) {
}

bool Switch::Get() {
	return !DigitalInput::Get();
}
