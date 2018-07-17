/*
 * exposurelimiter.h
 *
 *  Created on: Nov 15, 2015
 *      Author: matthias
 */

#ifndef SRC_AUTOADJUST_EXPOSURELIMITER_H_
#define SRC_AUTOADJUST_EXPOSURELIMITER_H_

#include <array>
#include "generic/helper.h"
#include "generic/types.h"
#include "generic/queuebuffer.h"
#include "configure/config.h"
#include "location/pose2d.h"

using namespace std;

namespace fml {

class ExposureLimiter {

	float translational_velocity_;	// current speed in m/s
	float translational_acceleration_;	// acceleration in m/s^2
	float angular_velocity_;	// current speed in rad/s
	float angular_acceleration_;	// acceleration in rad/s^2
	float pixel_speed_;		// pixel speed in pixel/s
	float f_mean_;			// mean exposure
	float global_min_distance_;	// distance to the closest possible marker in z-direction
	float max_pixel_speed_;	// static max allowed pixel speed (to determine max allowed speed dynamically)
	float max_shutter_fps_;	// max allowed shutter speed in µs due to fps
	map<int, float> cam_z_distance_;	// distance in z-direction in m for every marker id we know of
	// data structure to determine average speed over the last 1/3 second
	QueueBuffer<PoseHistoryEntry> position_history_;

	float computeSpeedPixelS(const list<int> &ids, const float max_center_distance);

public:
	ExposureLimiter();
	virtual ~ExposureLimiter();

	void addPose(const Pose2d &pose);
	void setVelocity(const float v_trans, const float v_rot);
	void reset();
	float getSpeedMSec() const;
	float getSpeedKmH() const;
	float getAngularVelocity() const;
	float getAccelMSec2() const;
	float getAccelRadSec2() const;
	float getPixelSpeed() const;
	float computeMaxExposure(const list<int> &ids, const float max_center_distance);
};

} /* namespace fml */

#endif /* SRC_AUTOADJUST_EXPOSURELIMITER_H_ */
