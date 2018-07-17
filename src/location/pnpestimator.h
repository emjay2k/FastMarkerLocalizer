/*
 * pnpestimator.h
 *
 *  Created on: 01.06.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_PNPESTIMATOR_H_
#define SRC_LOCATION_PNPESTIMATOR_H_

#include <opencv2/opencv.hpp>
#include "location/poseestimator.h"

using namespace std;

namespace fml {

class PnPEstimator: public PoseEstimator {

	int flags_;

public:
	PnPEstimator(const bool extrinsic_guess=false, const int flags=cv::ITERATIVE);
	virtual ~PnPEstimator();

	void estimatePose(list<MarkerInfo> &markers);
};

} /* namespace fml */

#endif /* SRC_LOCATION_PNPESTIMATOR_H_ */
