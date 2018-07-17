/*
 * kalmanlocationpredictor.cpp
 *
 *  Created on: 05.02.2015
 *      Author: jung
 */

#include "location/kalmanlocationpredictor.h"

namespace fml {

KalmanLocationPredictor::KalmanLocationPredictor(const float rk_x, const float rk_theta, const float qk_x, const float qk_theta) : KF_(6,3) {
	rk_x_ = rk_x;
	rk_theta_ = rk_theta;
	qk_x_ = qk_x;
	qk_theta_ = qk_theta;

	init();
}

KalmanLocationPredictor::~KalmanLocationPredictor() {
}

void KalmanLocationPredictor::init() {
	num_wraps_ = 0;
	predicted_angle_ = 0;

	// build kalman filter model
	// set F_k
	setDeltaT(1000.0f/Config::camera_parameters.fps);

	// set H_k
	KF_.measurementMatrix.at<float>(0,0) = 1;
	KF_.measurementMatrix.at<float>(1,1) = 1;
	KF_.measurementMatrix.at<float>(2,4) = 1;

	// set R_k
	float r_dxdx = rk_x_;
	float r_dtheta = rk_theta_;
	KF_.measurementNoiseCov.at<float>(0,0) = r_dxdx;	// dxdx
	KF_.measurementNoiseCov.at<float>(1,1) = r_dxdx;	// dydy = dxdx
	KF_.measurementNoiseCov.at<float>(2,2) = r_dtheta;	// dthetadtheta

	// set Q_k (this is camera system independent)
	float q_x_dot0 = qk_x_;
	float q_x_dot1 = sqrt_2 * q_x_dot0;
	float q_x_dot2 = 2 * q_x_dot0;
	float q_theta_dot0 = qk_theta_;
	float q_theta_dot1 = sqrt_2 * q_theta_dot0;
	float q_theta_dot2 = 2 * q_theta_dot0;
	KF_.processNoiseCov.at<float>(0,0) = q_x_dot0;
	KF_.processNoiseCov.at<float>(0,2) = q_x_dot1;
	KF_.processNoiseCov.at<float>(1,1) = q_x_dot0;
	KF_.processNoiseCov.at<float>(1,3) = q_x_dot1;
	KF_.processNoiseCov.at<float>(2,0) = q_x_dot1;
	KF_.processNoiseCov.at<float>(2,2) = q_x_dot2;
	KF_.processNoiseCov.at<float>(3,1) = q_x_dot1;
	KF_.processNoiseCov.at<float>(3,3) = q_x_dot2;
	KF_.processNoiseCov.at<float>(4,4) = q_theta_dot0;
	KF_.processNoiseCov.at<float>(4,5) = q_theta_dot1;
	KF_.processNoiseCov.at<float>(5,4) = q_theta_dot1;
	KF_.processNoiseCov.at<float>(5,5) = q_theta_dot2;
}

void KalmanLocationPredictor::setDeltaT(float delta_t) {
	KF_.transitionMatrix.at<float>(0,2) = delta_t;
	KF_.transitionMatrix.at<float>(1,3) = delta_t;
	KF_.transitionMatrix.at<float>(4,5) = delta_t;
}

/**
 * Get a prediction for time timestamp
 *
 * @param timestamp
 * @return
 */
Pose2d KalmanLocationPredictor::predict(const Pose2d &present_location, const clock_t timestamp) {
	if(!initialized_) {
		KF_.statePost.at<float>(0) = present_location.translation3d[0];
		KF_.statePost.at<float>(1) = present_location.translation3d[1];
		KF_.statePost.at<float>(2) = 0;	// no velocity along x
		KF_.statePost.at<float>(3) = 0;	// no velocity along y
		KF_.statePost.at<float>(4) = present_location.rotation;
		KF_.statePost.at<float>(5) = 0;	// no angular velocity
		initialized_ = true;
	} else {
		float delta_t = timestamp - previous_timestamp_;
		setDeltaT(delta_t);
	}
	previous_timestamp_ = timestamp;

	// predict next measurement
	cv::Mat prediction(KF_.predict());
	float x = prediction.at<float>(0);
	float y = prediction.at<float>(1);
	float yaw = prediction.at<float>(4);

	// normalize to obtain Euler angle and keep number of wraps we experienced
	num_wraps_ = TRUNC(yaw * 1.0f/g_PI);
	yaw -= num_wraps_ * g_PI2;
	predicted_angle_ = yaw;

	return(Pose2d(0, 0, timestamp, 0, x, y, Config::camera_parameters.z_offset, yaw));
}

/**
 * Update Kalman filters internal state and obtain estimation on current position in the process.
 *
 * @param present_location
 */
Pose2d KalmanLocationPredictor::update(const Pose2d &present_location) {
 	cv::Mat measurement(3,1,CV_32F);
	measurement.at<float>(0) = present_location.translation3d[0];
	measurement.at<float>(1) = present_location.translation3d[1];

	// normalize angle for correction
	float yaw_update = present_location.rotation;
	// check if we have a change in the number of wraps
	float diff = predicted_angle_ - yaw_update;
	num_wraps_ -= (diff < -g_PI);
	num_wraps_ += (diff > g_PI);

	yaw_update += num_wraps_ * g_PI2;
	measurement.at<float>(2) = yaw_update;

	// have kalman filter correct it
	cv::Mat estimated(KF_.correct(measurement));
	// this is our best guess
	float x = estimated.at<float>(0);
	float y = estimated.at<float>(1);
	float xdot = estimated.at<float>(2);
	float ydot = estimated.at<float>(3);
	float yaw = estimated.at<float>(4);
	float yawdot = estimated.at<float>(5);

	// normalize it to obtain a usable Euler angle
	yaw -= TRUNC(yaw * 1.0f/g_PI) * g_PI2;

	// update velocity values
	velocity_ = SQRT(xdot*xdot + ydot*ydot);
	angular_velocity_ = 1000 * yawdot;

	return(Pose2d(0, 0, present_location.timestamp, 0,  x, y, Config::camera_parameters.z_offset, yaw));
}

void KalmanLocationPredictor::reset() {
	LocationPredictor::reset();
	KF_.init(6,3);	// clear the collected
	init();	// setup our own data
}

} /* namespace fml */
