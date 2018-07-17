/*
 * exposurelimiter.cpp
 *
 *  Created on: Nov 15, 2015
 *      Author: matthias
 */

#include "autoadjust/exposurelimiter.h"

namespace fml {

ExposureLimiter::ExposureLimiter() : position_history_(2) {
	f_mean_ = 0.5f * (Config::calibration_parameters.camera_matrix.at<float>(0,0) + Config::calibration_parameters.camera_matrix.at<float>(1,1));
	// defaults to ridiculously high value, so we can detect whether we can actually determine acceleration values
	translational_velocity_ = FLT_MAX;
	translational_acceleration_ = FLT_MAX;
	angular_velocity_ = FLT_MAX;
	angular_acceleration_ = FLT_MAX;
	pixel_speed_ = FLT_MAX;
	max_pixel_speed_= Config::camera_parameters.pixel_speedlimit;
	max_shutter_fps_ = 1000000.0f/Config::camera_parameters.fps;

	global_min_distance_ = FLT_MAX;
	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) {
		float distance = (float)(1.0/1000.0) * (it->second.translation3d[2] - Config::camera_parameters.z_offset);	// distance in m
		cam_z_distance_.insert(pair<int, float>(it->first, distance));
		if(distance < global_min_distance_) {
			global_min_distance_ = distance;
		}
	}
}

ExposureLimiter::~ExposureLimiter() {
}

// compute the speed of the marker pixels in the image (in pixel/s)
// this is a simplification: theoretically translation and rotational speeds can be additive to the pixel movement,
// however a forklift can only have both high translational or high angular velocities, so we neglect the lower part.
// in cases of moderate velocities this feature is unnecessary anyway. If you want to do this correctly, you have
// to compute the resulting vector of the translational and rotational velocity in the pixel domain and find the maximum
// for all markers in the camera's fov - it's a waste of time for forklifts, though.
float ExposureLimiter::computeSpeedPixelS(const list<int> &ids, const float max_center_distance) {
	// translation
	// the min distance to a marker in z-direction
	float z_min = FLT_MAX;
	for(list<int>::const_iterator it = ids.begin(); it != ids.end(); ++it) {
		if(cam_z_distance_.count(*it)) {
			float z = cam_z_distance_.at(*it);
			if(z < z_min) {
				z_min = z;
			}
		}
	}
	if(z_min == FLT_MAX) {
		z_min = global_min_distance_;
	}
	float v_trans = f_mean_/z_min * getSpeedMSec();

	// rotation
	float v_rot = max_center_distance * getAngularVelocity();
	pixel_speed_ = SQRT(v_trans*v_trans + v_rot*v_rot);
	return(pixel_speed_);
}

// enqueue current position data into history
void ExposureLimiter::addPose(const Pose2d &pose) {
	position_history_.enqueue(PoseHistoryEntry(pose.timestamp, pose.translation3d[0], pose.translation3d[1], pose.rotation));
	// update speed in m/s
	if(position_history_.isFull()) {
		PoseHistoryEntry first = position_history_.front();
		PoseHistoryEntry last = position_history_.back();
		float rdt = 1.0f/(first.timestamp - last.timestamp);
		// translation
		float dx = first.x - last.x;
		float dy = first.y - last.y;
		float v_trans = SQRT(dx*dx + dy*dy)*rdt;
		// rotation
		float dtheta = g_PI - FABS(g_PI - FABS(first.angle - last.angle));
		float v_rot = FABS(dtheta*1000.0f*rdt);
		// initially we cannot determine the acceleration as we do not yet have the velocity
		if(likely(translational_velocity_ < FLT_MAX)) {
			translational_acceleration_ =  v_trans - translational_velocity_;
			angular_acceleration_ = v_rot - angular_velocity_;
		} else {
			translational_acceleration_ = 0;
			angular_acceleration_ = 0;
		}

		translational_velocity_ = v_trans;
		angular_velocity_ = v_rot;
	}
}

void ExposureLimiter::setVelocity(const float v_trans, const float v_rot) {
	translational_acceleration_ = (translational_velocity_ < FLT_MAX) * (v_trans - translational_velocity_);
	translational_velocity_ = v_trans;
	angular_acceleration_ = (angular_velocity_ < FLT_MAX) * (v_rot - angular_velocity_);
	angular_velocity_ = v_rot;

	position_history_.free();
}

// reset after prolonged loss of positioning
void ExposureLimiter::reset() {
	translational_velocity_ = FLT_MAX;
	translational_acceleration_ = FLT_MAX;
	angular_velocity_ = FLT_MAX;
	angular_acceleration_ = FLT_MAX;
	position_history_.free();
}

// return the physical speed of the camera in the scene in m/s
float ExposureLimiter::getSpeedMSec() const {
	return(likely(translational_velocity_ < FLT_MAX) ? translational_velocity_ : 0);
}

// return the physical speed of the camera in the scene in km/h
float ExposureLimiter::getSpeedKmH() const {
	return(getSpeedMSec()*3.6f);
}

float ExposureLimiter::getAngularVelocity() const {
	return(likely(angular_velocity_ < FLT_MAX) ? angular_velocity_ : 0);
}

float ExposureLimiter::getAccelMSec2() const {
	return(likely(translational_acceleration_ < FLT_MAX) ? translational_acceleration_ : 0);
}

float ExposureLimiter::getAccelRadSec2() const {
	return(likely(angular_acceleration_ < FLT_MAX) ? angular_acceleration_ : 0);
}

float ExposureLimiter::getPixelSpeed() const {
	return(pixel_speed_);
}

// determine the max shutter speed based on the translational and rotational pixel speed
float ExposureLimiter::computeMaxExposure(const list<int> &ids, const float max_center_distance) {
	float pixel_speed = computeSpeedPixelS(ids, max_center_distance) + FLT_EPSILON;
	//cout << ": pixel_speed=" << pixel_speed << endl;
	return(min(1000000.0f * (max_pixel_speed_/pixel_speed), max_shutter_fps_));
}

} /* namespace fml */
