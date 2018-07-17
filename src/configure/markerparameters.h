/*
 * markerparameters.h
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#ifndef MARKERPARAMETERS_H_
#define MARKERPARAMETERS_H_

#include <map>
#include <opencv2/opencv.hpp>
#include "generic/definitions.h"
#include "location/pose2d.h"

using namespace std;

namespace fml {

class MarkerParameters {

public:
	map<int, Pose2d> base_T_m;	// object pose in world frame mapped to camera or marker id
	map<int, array<float, 2> > rotation_xy;

	MarkerParameters();
	MarkerParameters(const string &filename);
	virtual ~MarkerParameters();

	void write(const string &filename) const;
};

} /* namespace fml */

#endif /* MARKERPARAMETERS_H_ */
