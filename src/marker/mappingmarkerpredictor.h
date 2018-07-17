/*
 * MappingMarkerPredictor.h
 *
 *  Created on: 19.01.2015
 *      Author: jung
 */

#ifndef MAPPINGMARKERPREDICTOR_H_
#define MAPPINGMARKERPREDICTOR_H_

#include <algorithm>
#include <map>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>
#include "configure/config.h"
#include "generic/definitions.h"
#include "generic/helper.h"
#include "location/vectorlocationpredictor.h"
#include "marker/roipredictor.h"
#include "marker/marker.h"
#include "marker/undistorter.h"

using namespace std;

namespace fml {

class MappingMarkerPredictor {
	map<int, float> marker_heights_;	// marker heights above camera
	map<float, MarkerProjectedCorners*> projected_corners_by_size_;	// projected corners by size of the markers
	int num_rois_;
	int num_rois_completely_inside_;

	map<int, VectorLocationPredictor> trackers_;
	map<int, map<int, Pose2d> > mapping_relations_;
	list<GraphVertex> nodes_;	// adjacency list
	list<GraphVertex>::iterator p_vertices_[1024];	// pointers to elements in nodes for easier access of the adjacency list

	cv::Point3f pointCoordinate(const Pose2d &lhs, const Pose2d &rhs) const;	// multiply lhs and rhs but only return the translation part

	void getNeighbors(const int i, list<int> &neighbors) const;
	void addNeighbor(const int i, const int j);
	void addRelation(const int src, const int dst, Pose2d &m1_T_m2);

	void DFS(const int vertex_id, const int c);
	void resetAdjacencyList();

public:
	MappingMarkerPredictor();
	virtual ~MappingMarkerPredictor();

	void addRelations(const list<MarkerInfo> &c_T_m);
	void getROIs(const list<MarkerInfo> &c_T_m, vector<MarkerROI> &rois);
	void getTrackedROIs(const list<MarkerInfo> &c_T_m, vector<MarkerROI> &rois, const int timestamp);
	void updateTrackers(list<MarkerInfo> &c_T_m, const int timestamp);
	list<int> getMarkersNotConnected();
};

}

#endif /* MAPPINGMARKERPREDICTOR_H_ */
