/*
 * kalmanlocationpredictor.h
 *
 *	If you ever find yourself in the position to do sensor fusion in this framework,
 *	this is the place to integrate it. Currently it is only used for predicting and refining
 *	positions based on past information (which it does pretty good).
 *
 *  Created on: 05.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_KALMANLOCATIONPREDICTOR_H_
#define SRC_LOCATION_KALMANLOCATIONPREDICTOR_H_

#include <boost/array.hpp>
#include <opencv2/opencv.hpp>
#include "configure/config.h"
#include "generic/definitions.h"
#include "generic/helper.h"
#include "location/locationpredictor.h"

namespace fml {

class KalmanLocationPredictor : public LocationPredictor {
	clock_t previous_timestamp_;
	cv::KalmanFilter KF_;
	// R_k
	float rk_x_;
	float rk_theta_;
	// Q_k
	float qk_x_;
	float qk_theta_;

	float num_wraps_;
	float predicted_angle_;

	void init();
	void setDeltaT(float delta_t);

public:
	KalmanLocationPredictor(const float rk_x, const float rk_theta, const float qk_x, const float qk_theta);
	virtual ~KalmanLocationPredictor();

	Pose2d predict(const Pose2d &present_location, const clock_t timestamp);
	Pose2d update(const Pose2d &present_location);
	void reset();
};

} /* namespace fml */

#endif /* SRC_LOCATION_KALMANLOCATIONPREDICTOR_H_ */
