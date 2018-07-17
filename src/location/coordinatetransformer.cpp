/*
 * coordinatetransformer.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include "location/coordinatetransformer.h"

namespace fml {

CoordinateTransformer::CoordinateTransformer() {
	Pose2d camera_to_vehicle(0, 0, 0, 0, Config::camera_parameters.x_offset, Config::camera_parameters.y_offset, Config::camera_parameters.z_offset, Config::camera_parameters.angle_offset);
	setVehicleTransformation(camera_to_vehicle);
}

CoordinateTransformer::CoordinateTransformer(const Pose2d &v_T_c) {
	setVehicleTransformation(v_T_c);
}

CoordinateTransformer::~CoordinateTransformer() {
}

/**
 * Set transformation from camera to vehicle and vice versa
 * @param v_T_c
 */
void CoordinateTransformer::setVehicleTransformation(const Pose2d &v_T_c) {
	v_T_c_ = v_T_c;
	c_T_v_ = v_T_c;
	c_T_v_.invert();
}

/**
 * Transform marker in camera to camera in world
 *
 * @param c_T_m
 * @return
 */
bool CoordinateTransformer::toWorld(Pose2d &c_T_m) const {
	bool possible = Config::marker_parameters.base_T_m.count(c_T_m.id);
	if(possible) {
		//cout << "before," << c_T_m.translation3d[0] << "," << c_T_m.translation3d[1] << "," << c_T_m.rotation3d[2] << endl;
		c_T_m.invert();	// turn it into m_T_c
		//c_T_m.printMatrix();
		//cout << "after," << c_T_m.translation3d[0] << "," << c_T_m.translation3d[1] << "," << c_T_m.rotation3d[2] << endl;
		c_T_m = Config::marker_parameters.base_T_m.at(c_T_m.id) * c_T_m;
		// result is actually w_T_c
	}
	return(possible);
}

/**
 * Transform camera in world to marker in camera
 *
 * @param c_T_m
 * @return
 */
bool CoordinateTransformer::toMarker(Pose2d &w_T_c) const {
	bool possible = Config::marker_parameters.base_T_m.count(w_T_c.id);
	if(possible) {
		w_T_c.invert();	// turn it into c_T_w
		w_T_c = w_T_c * Config::marker_parameters.base_T_m.at(w_T_c.id);
		// result is actually c_T_m
	}
	return(possible);
}

/**
 * Transform camera in world to vehicle in world
 *
 * @param w_T_c
 * @return
 */
void CoordinateTransformer::toVehicle(Pose2d &w_T_c) const {
	w_T_c = w_T_c * c_T_v_;
}

/**
 * Transform vehicle in world to camera in world
 *
 * @param w_T_v
 * @return
 */
void CoordinateTransformer::toCamera(Pose2d &w_T_v) const {
	w_T_v = w_T_v * v_T_c_;
}


/**
 * Computes the pose between the marker coordinate systems
 *
 * @param c_T_m1 first coordinate system
 * @param c_T_m2 second coordinate system
 * @param m1_T_m2 the result (m2 in m1 frame)
 */
void CoordinateTransformer::markerToMarker(Pose2d &c_T_m1, const Pose2d &c_T_m2, Pose2d &m1_T_m2) const {
	c_T_m1.invert(); // this is now m1_T_c
	m1_T_m2 = c_T_m1 * c_T_m2;
}

} /* namespace fml */
