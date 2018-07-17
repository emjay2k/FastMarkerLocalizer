/*
 * markerparameters.cpp
 *
 *  Created on: 12.09.2014
 *      Author: jung
 */

#include "configure/markerparameters.h"

namespace fml {

MarkerParameters::MarkerParameters() {
}

MarkerParameters::MarkerParameters(const string &filename) {
	cv::Mat poses, rotations;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["Poses"] >> poses;
	fs["Rotations"] >> rotations;
	fs.release();
	// read all saved poses, which are stored inside a matrix
	for(int i = 0; i < poses.rows; ++i) {
		const float* marker_row = poses.ptr<float>(i);
		int id = (int)marker_row[0];	// id of marker or source
		base_T_m.insert(pair<int, Pose2d>(id, Pose2d(id, id, 0, 0, marker_row[1], marker_row[2], marker_row[3], marker_row[4])));
	}
	for(int i = 0; i < rotations.rows; ++i) {
		const float* marker_row = rotations.ptr<float>(i);
		int id = (int)marker_row[0];	// marker id
		array<float, 2> rotation;
		rotation[0] = marker_row[1];
		rotation[1] = marker_row[2];
		rotation_xy.insert(pair<int, array<float,2>>(id,rotation));
	}
}

MarkerParameters::~MarkerParameters() {
}

void MarkerParameters::write(const string &filename) const {
	if(!base_T_m.empty()) {
		cv::Mat poses(base_T_m.size(), 5, MAT_TYPE);
		// write all poses from map into matrix and then to file
		int row = 0;
		for(map<int, Pose2d>::const_iterator it = base_T_m.begin(); it != base_T_m.end(); ++it) {
			float* marker_row = poses.ptr<float>(row++);
			marker_row[0] = (float)(it->first);
			marker_row[1] = it->second.translation3d[0];
			marker_row[2] = it->second.translation3d[1];
			marker_row[3] = it->second.translation3d[2];
			marker_row[4] = it->second.rotation;
		}
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
		fs << "Poses" << poses;
		fs.release();
	}
}

} /* namespace fml */
