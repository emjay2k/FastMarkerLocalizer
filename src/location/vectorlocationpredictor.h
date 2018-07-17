/*
 * vectorlocationpredictor.h
 *
 *  Created on: 05.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_VECTORLOCATIONPREDICTOR_H_
#define SRC_LOCATION_VECTORLOCATIONPREDICTOR_H_

#include <boost/array.hpp>
#include "generic/definitions.h"
#include "generic/helper.h"
#include "location/locationpredictor.h"

namespace fml {

class VectorLocationPredictor : public LocationPredictor {
	bool fully_operational_;
	int lost_;
	Pose2d past_location_;
	Pose2d past_past_location_;

public:
	VectorLocationPredictor();
	virtual ~VectorLocationPredictor();

	Pose2d predict(const Pose2d &present_location, const clock_t timestamp);
	// rest is for mapping prediction
	int getLost() const;
	void setLost(const int lost);
	Pose2d predict(const clock_t timestamp);
	Pose2d update(const Pose2d &present_location);
	void updatePoses(const Pose2d &present_location);
};

} /* namespace fml */

#endif /* SRC_LOCATION_VECTORLOCATIONPREDICTOR_H_ */
