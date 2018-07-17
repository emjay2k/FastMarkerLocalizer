/*
 * locationpredictor.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATIONPREDICTOR_H_
#define SRC_LOCATIONPREDICTOR_H_

#include "location/pose2d.h"

namespace fml {

class LocationPredictor {

protected:
	bool initialized_;
	float velocity_;		 // current translational velocity in m/s
	float angular_velocity_; // current angular velocity in rad/s
	float angular_acceleration_;	// current angular acceleration in rad/s^2

public:
	LocationPredictor();
	virtual ~LocationPredictor();

	virtual Pose2d predict(const Pose2d &present_location, const clock_t timestamp) = 0;
	virtual Pose2d update(const Pose2d &present_location) = 0;
	void reset();

	float getVelocityMSec() const;
	float getAngularVelocity() const;
	float getAngularAcceleration() const;
};

} /* namespace fml */

#endif /* SRC_LOCATIONPREDICTOR_H_ */
