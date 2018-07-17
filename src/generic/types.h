/**
 * types.h
 *
 *  Created on: 13.02.2012
 *      Author: jung
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <ctime>
#include <opencv2/opencv.hpp>
#include "location/pose2d.h"

using namespace std;

namespace fml {

// not all of the enum configurations are necessarily implemented, but could be integrated at a later time
enum class MarkerMode { ARUCO_FID, ARUCO_HRM };
enum class TrackerAlgorithm { TRACK_VECTOR, TRACK_KALMAN, TRACK_EXTKALMAN };
enum class PoseEstimationAlgorithm { POSE_PPP, POSE_RPP, POSE_ITERATIVE, POSE_EPNP, POSE_P3P, POSE_POSIT, POSE_PLANARPOSIT };

struct PoseHistoryEntry {
	clock_t timestamp;
	float x;
	float y;
	float angle;

	PoseHistoryEntry(const clock_t timestamp, const float x, const float y, const float angle) {
		this->timestamp = timestamp;
		this->x = x;
		this->y = y;
		this->angle = angle;
	}
};

struct DijkstraNode {
	int id;
	int pred;
	float distance;

	DijkstraNode(const int id) {
		this->id = id;
		this->pred = -1;
		distance = FLT_MAX;
	}

	bool operator==(const DijkstraNode &rhs) const {
		return(this->id == rhs.id);
	}
};

struct GraphNeighbor {
	int id;
	int count;

	GraphNeighbor(const int id) {
		this->id = id;
		count = 0;
	}
};

struct GraphVertex {
	int id;
	int component;
	list<GraphNeighbor> neighbors;

	GraphVertex(const int id) {
		this->id = id;
		component = 0;
	}
};

struct GraphEdge {
	int src;
	int dst;
	float cost;
	Pose2d relation;

	GraphEdge(const int src, const int dst, const float cost, const float x, const float y, const float z, const float yaw) {
		this->src = src;
		this->dst = dst;
		this->cost = cost;
		relation = Pose2d(dst, src, 0, cost , x, y, z, yaw);
	}

	bool operator<(GraphEdge const &rhs) const {
		return(this->cost < rhs.cost);
	}
};

} /* namespace fml */

#endif /* TYPES_H_ */
