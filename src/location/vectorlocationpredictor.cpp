/*
 * vectorlocationpredictor.cpp
 *
 *  Created on: 05.02.2015
 *      Author: jung
 */

#include "location/vectorlocationpredictor.h"

namespace fml {

VectorLocationPredictor::VectorLocationPredictor() {
	fully_operational_ = false;
	lost_ = 0;
}

VectorLocationPredictor::~VectorLocationPredictor() {
}

int VectorLocationPredictor::getLost() const {
	return(lost_);
}

void VectorLocationPredictor::setLost(const int lost) {
	lost_ = lost;
}

/**
 * This method is very acurate in describing the next position. Major error sources are
 *
 * previous pose error (to be measured), probably in the range of 4-5cm
 * Acceleration or deceleration (negligible, s = +- 0.5 * a * fps^-2). Can be approximated by 1cm static error for 20fps
 * This does not account for accidents (forklift hitting an indestructible wall at full speed), in that case the pose deviation is the least of the companys problems
 *
 * @param present_location
 * @param timestamp
 * @return
 */
Pose2d VectorLocationPredictor::predict(const Pose2d &present_location, const clock_t timestamp) {
	if(initialized_) {
		float translation3d[3];
		float rotation = 0;
		float factor = (float)(timestamp - present_location.timestamp)/(float)(present_location.timestamp - past_location_.timestamp);
		// translation
		translation3d[0] = present_location.translation3d[0] + factor * (present_location.translation3d[0] - past_location_.translation3d[0]);
		translation3d[1] = present_location.translation3d[1] + factor * (present_location.translation3d[1] - past_location_.translation3d[1]);
		translation3d[2] = present_location.translation3d[2] + factor * (present_location.translation3d[2] - past_location_.translation3d[2]);

		// rotation
		// switch to polar coordinates and move back
		array<float, 2> present_polar_angle = toPolar(present_location.rotation);
		array<float, 2> past_polar_angle = toPolar(past_location_.rotation);
		// t-1
		float x = present_polar_angle[0] + factor * (present_polar_angle[0] - past_polar_angle[0]);
		float y = present_polar_angle[1] + factor * (present_polar_angle[1] - past_polar_angle[1]);
		rotation = ATAN2(y, x);

		past_location_ = present_location;
		return(Pose2d(present_location.id, present_location.source_id, timestamp, 0, translation3d[0], translation3d[1], translation3d[2], rotation));
	} else {
		past_location_ = present_location;
		initialized_ = true;
		return(Pose2d(present_location.id, present_location.source_id, timestamp, 0, present_location.translation3d[0], present_location.translation3d[1], present_location.translation3d[2], present_location.rotation));
	}
}

Pose2d VectorLocationPredictor::predict(const clock_t timestamp) {
	if(fully_operational_) {
		// actual vector location prediction
		float translation3d[3];
		float rotation = 0;
		float factor = (float)(timestamp - past_location_.timestamp)/(float)(past_location_.timestamp - past_past_location_.timestamp);
		// translation
		translation3d[0] = past_location_.translation3d[0] + factor * (past_location_.translation3d[0] - past_past_location_.translation3d[0]);
		translation3d[1] = past_location_.translation3d[1] + factor * (past_location_.translation3d[1] - past_past_location_.translation3d[1]);
		translation3d[2] = past_location_.translation3d[2] + factor * (past_location_.translation3d[2] - past_past_location_.translation3d[2]);

		// rotation
		// switch to polar coordinates and move back
		array<float, 2> past_polar_angle = toPolar(past_location_.rotation);
		array<float, 2> past_past_polar_angle = toPolar(past_past_location_.rotation);
		// t-1
		float x = past_polar_angle[0] + factor * (past_polar_angle[0] - past_past_polar_angle[0]);
		float y = past_polar_angle[1] + factor * (past_polar_angle[1] - past_past_polar_angle[1]);
		rotation = ATAN2(y, x);

		return(Pose2d(past_location_.id, past_location_.source_id, timestamp, 0, translation3d[0], translation3d[1], translation3d[2], rotation));
	} else if(initialized_) {
		// hope we didn't move far (this is one of the reasons you have to drive slowly during mapping)
		return(Pose2d(past_location_.id, past_location_.source_id, timestamp, 0, past_location_.translation3d[0], past_location_.translation3d[1], past_location_.translation3d[2], past_location_.rotation));
	} else {
		return(Pose2d());
	}
}

/**
 * We do not have the ability to refine our measurement in this predictor
 *
 * @param present_location
 * @return
 */
Pose2d VectorLocationPredictor::update(const Pose2d &present_location) {
	float dx = present_location.translation3d[0] - past_location_.translation3d[0];
	float dy = present_location.translation3d[1] - past_location_.translation3d[1];
	float dt = present_location.timestamp - past_location_.timestamp;
	float rdt = (initialized_ && dt != 0) ? 1.0f/dt : 0;
	velocity_ = SQRT(dx*dx + dy*dy) * rdt;

	float dtheta = present_location.rotation - past_location_.rotation;
	dtheta += (dtheta > g_PI) * -g_PI2;
	dtheta += (dtheta < -g_PI) * g_PI2;
	angular_velocity_ = dtheta * 1000 * rdt;

	return(present_location);
}

/**
 * Second use case: relative tracking during mapping. Update status.
 *
 * @param present_location
 */
void VectorLocationPredictor::updatePoses(const Pose2d &present_location) {
	if(initialized_) {
		fully_operational_ = true;	// we are now/still fully operational
		past_past_location_ = past_location_;
	} else {
		initialized_ = true;
	}
	past_location_ = present_location;
}

} /* namespace fml */
