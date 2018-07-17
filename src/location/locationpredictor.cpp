/*
 * locationpredictor.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "location/locationpredictor.h"

namespace fml {

LocationPredictor::LocationPredictor() {
	initialized_ = false;
	velocity_ = 0;
	angular_velocity_ = 0;
	angular_acceleration_ = 0;
}

LocationPredictor::~LocationPredictor() {
}

void LocationPredictor::reset() {
	initialized_ = false;
	velocity_ = 0;
	angular_velocity_ = 0;
}

float LocationPredictor::getVelocityMSec() const {
	return(velocity_);
}

float LocationPredictor::getAngularVelocity() const {
	return(angular_velocity_);
}

float LocationPredictor::getAngularAcceleration() const {
	return(angular_acceleration_);
}

} /* namespace fml */
