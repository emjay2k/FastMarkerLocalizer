/*
 * roipredictor.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_ROIPREDICTOR_H_
#define SRC_ROIPREDICTOR_H_

#include "location/pose2d.h"
#include "marker/marker.h"

using namespace std;

namespace fml {

class RoiPredictor {

public:
	RoiPredictor();
	virtual ~RoiPredictor();

	virtual bool predict(const Pose2d &w_T_c, vector<MarkerROI> &rois) = 0;
};

} /* namespace fml */

#endif /* SRC_ROIPREDICTOR_H_ */
