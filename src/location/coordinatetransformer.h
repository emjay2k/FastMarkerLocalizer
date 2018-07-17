/*
 * coordinatetransformer.h
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#ifndef SRC_COORDINATETRANSFORMER_H_
#define SRC_COORDINATETRANSFORMER_H_

#include "location/pose2d.h"
#include <configure/config.h>

namespace fml {

class CoordinateTransformer {
	Pose2d v_T_c_;	// camera in vehicle coordinates
	Pose2d c_T_v_;	// (v_T_c)^-1

public:
	CoordinateTransformer();
	CoordinateTransformer(const Pose2d &v_T_c);
	virtual ~CoordinateTransformer();

	void setVehicleTransformation(const Pose2d &v_T_c);

	bool toWorld(Pose2d &c_T_m) const;
	bool toMarker(Pose2d &w_T_c) const;
	void toVehicle(Pose2d &w_T_c) const;
	void toCamera(Pose2d &w_T_v) const;
	void markerToMarker(Pose2d &c_T_m1, const Pose2d &c_T_m2, Pose2d &m1_T_m2) const;
};

} /* namespace fml */

#endif /* SRC_COORDINATETRANSFORMER_H_ */
