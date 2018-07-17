/*
 * MappingMarkerPredictor.cpp
 *
 *  Created on: 19.01.2015
 *      Author: jung
 */

#include "marker/mappingmarkerpredictor.h"

namespace fml {

MappingMarkerPredictor::MappingMarkerPredictor() {
	// maximum error of the projected pose in each direction
	const float max_error_mm = 100;

	nodes_.push_back(GraphVertex(Config::tracker_parameters.mapping_startid));
	list<GraphVertex>::iterator graph_it = nodes_.begin();
	p_vertices_[Config::tracker_parameters.mapping_startid] = graph_it;
	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) {
		marker_heights_.insert(pair<int, float>(it->first, it->second.translation3d[2] - Config::camera_parameters.z_offset));
		if(it->first != Config::tracker_parameters.mapping_startid) {
			nodes_.push_back(GraphVertex(it->first));
			p_vertices_[it->first] = ++graph_it;
		}
	}

	// determine precomputed marker corners
	// additional 9.0/7.0 factor is because we need the white border of the marker for edge detection as well, whilst the marker size only covers the black contents.
	projected_corners_by_size_.insert(pair<float, MarkerProjectedCorners*>(Config::tracker_parameters.marker_size_mm, new MarkerProjectedCorners(max_error_mm + (float)(9.0/14.0) * Config::tracker_parameters.marker_size_mm, 0.5f * Config::tracker_parameters.marker_size_mm)));
	for(map<int, float>::const_iterator it = Config::tracker_parameters.special_marker_sizes_mm.begin(); it != Config::tracker_parameters.special_marker_sizes_mm.end(); ++it) {
		// add new one if we do not have this one yet,
		if(projected_corners_by_size_.count(it->second) == 0) {
			projected_corners_by_size_.insert(pair<float, MarkerProjectedCorners*>(it->second, new MarkerProjectedCorners(max_error_mm + (float)(9.0/14.0) * it->second, 0.5f * it->second)));
		}
	}

	num_rois_ = 0;
	num_rois_completely_inside_ = 0;
}

MappingMarkerPredictor::~MappingMarkerPredictor() {
	cerr << "mapping: number_of_rois considered: " << num_rois_ << endl;
	cerr << "mapping: number_of_rois completely in image: " << num_rois_completely_inside_ << endl;

	// cleanup the map with the precomputed marker corners ready for projection
	for(map<float, MarkerProjectedCorners*>::iterator it = projected_corners_by_size_.begin(); it != projected_corners_by_size_.end(); ++it) {
		delete it->second;
	}
}

/**
 * Saves us some Pose products
 *
 * @param lhs
 * @param rhs
 * @return
 */
cv::Point3f MappingMarkerPredictor::pointCoordinate(const Pose2d &lhs, const Pose2d &rhs) const {
	float x_left = lhs.transformation[0];
	float y_left = lhs.transformation[4+0];

	float x = lhs.transformation[3] + rhs.transformation[3] * x_left - rhs.transformation[4+3] * y_left;
	float y = lhs.transformation[4+3] + rhs.transformation[4+3] * x_left + rhs.transformation[3] * y_left;
	float z = lhs.transformation[8+3] + rhs.transformation[8+3];

	return(cv::Point3f(x, y, z));
}

void MappingMarkerPredictor::DFS(int vertex_id, int c) {
	p_vertices_[vertex_id]->component = c;
	for(list<GraphNeighbor>::const_iterator n_it = p_vertices_[vertex_id]->neighbors.begin(); n_it != p_vertices_[vertex_id]->neighbors.end(); ++n_it) {
		if(n_it->count > Config::tracker_parameters.mapping_minmeasurements && p_vertices_[n_it->id]->component == 0) {
			DFS(n_it->id, c);
		}
	}
}

void MappingMarkerPredictor::resetAdjacencyList() {
	for(list<GraphVertex>::iterator graph_it = nodes_.begin(); graph_it != nodes_.end(); ++graph_it) {
		graph_it->component = 0;
	}
}

list<int> MappingMarkerPredictor::getMarkersNotConnected() {
	resetAdjacencyList();
	list<int> retval;
	// run dfs
	int c = 0;
	for(list<GraphVertex>::const_iterator graph_it = nodes_.begin(); graph_it != nodes_.end(); ++graph_it) {
		if(graph_it->component == 0) {
			++c;
			DFS(graph_it->id, c);
		}
	}
	// return the markers belonging to components with id > 1 (which is the one with the start_id in it)
	for(list<GraphVertex>::const_iterator graph_it = nodes_.begin(); graph_it != nodes_.end(); ++graph_it) {
		if(graph_it->component > 1) {
			retval.push_back(graph_it->id);
		}
	}

	return(retval);
}

void MappingMarkerPredictor::getNeighbors(const int i, list<int> &neighbors) const {
	for(list<GraphNeighbor>::const_iterator n_it = p_vertices_[i]->neighbors.begin(); n_it != p_vertices_[i]->neighbors.end(); ++n_it) {
		neighbors.push_back(n_it->id);
	}
}

void MappingMarkerPredictor::addNeighbor(const int i, const int j) {
	// now the dfs data structure (which is partially redundant
	for(list<GraphNeighbor>::iterator n_it = p_vertices_[i]->neighbors.begin(); n_it != p_vertices_[i]->neighbors.end(); ++n_it) {
		if(n_it->id == j) {
			++(n_it->count);
			return;
		}
	}
	// if not returned above, we have to add the neighbor to the list
	p_vertices_[i]->neighbors.push_back(GraphNeighbor(j));
}


void MappingMarkerPredictor::addRelation(const int src, const int dst, Pose2d &m1_T_m2) {
	if(mapping_relations_.count(src) == 0) {
		mapping_relations_.insert(pair<int, map<int, Pose2d> >(src, map<int, Pose2d>()));
	}
	if(mapping_relations_.at(src).count(dst) == 0) {	// new relation
		m1_T_m2.count = 1;
		mapping_relations_.at(src).insert(pair<int, Pose2d>(dst, m1_T_m2));
	} else {
		// merge new measurement with previous mean
		list<Pose2d> poses;
		poses.push_back(mapping_relations_.at(src).at(dst));
		poses.push_back(m1_T_m2);
		float old_count = (float)poses.front().count;
		vector<float> weights(2);
		weights[0] = old_count;
		weights[1] = 1.0f;
		Pose2d newPose(weightedMeanPose(poses, weights));
		newPose.count = (int)old_count + 1;
		mapping_relations_.at(src).at(dst) = newPose;
	}
}

void MappingMarkerPredictor::addRelations(const list<MarkerInfo> &markers) {
	// we need more than one marker to obtain relations
	if(markers.size() > 1) {
		list<MarkerInfo>::const_iterator end1 = markers.end();
		--end1;
		for(list<MarkerInfo>::const_iterator it1 = markers.begin(); it1 != end1; ++it1) {
			list<MarkerInfo>::const_iterator it2 = it1;
			++it2;
			for(; it2 != markers.end(); ++it2) {
				addNeighbor(it1->id, it2->id);
				addNeighbor(it2->id, it1->id);
				Pose2d pose1, pose2;
				int src, dst;
				if(it1->id < it2->id) {
					src = it1->id;
					dst = it2->id;
					pose1 = it1->w_T_m;	// c_T_m1
					pose2 = it2->w_T_m;	// c_T_m2
				} else {
					src = it2->id;
					dst = it1->id;
					pose1 = it2->w_T_m;	// c_T_m1
					pose2 = it1->w_T_m;	// c_T_m2
				}

				pose1.invert(); // make it m1_T_c
				Pose2d m1_T_m2(pose1 * pose2);	// m1_T_m2
				addRelation(src, dst, m1_T_m2);
			}
		}
	}
}

void MappingMarkerPredictor::updateTrackers(list<MarkerInfo> &c_T_m, const int timestamp) {
	const int max_loss = 2;
	set<int> found_ids;
	for(list<MarkerInfo>::iterator it = c_T_m.begin(); it != c_T_m.end(); ++it) {
		found_ids.insert(it->id);
		if(trackers_.count(it->id) == 0) {	// insert
			trackers_.insert(pair<int, VectorLocationPredictor>(it->id, VectorLocationPredictor()));
		}
		it->w_T_m.timestamp = timestamp;
		trackers_.at(it->id).updatePoses(it->w_T_m);	// update/initialize
		trackers_.at(it->id).setLost(0);
	}

	// remove the ones we lost track of
	for(map<int, VectorLocationPredictor>::iterator it = trackers_.begin(); it != trackers_.end();) {
		if(found_ids.count(it->first) == 0) { // erase
			it->second.setLost(it->second.getLost() + 1);
			if(it->second.getLost() >= max_loss) {
				it = trackers_.erase(it);
			}
		} else {
			++it;
		}
	}
}

/**
 *
 * @param c_T_m Poses of the found markers
 * @param rois return values: rois of further markers
 */
void MappingMarkerPredictor::getROIs(const list<MarkerInfo> &markers, vector<MarkerROI> &rois) {
	rois.clear();

	set<int> found;
	list<Pose2d> additional_poses;	// c_T_x (in camera coordinates)

	for(list<MarkerInfo>::const_iterator pose_it = markers.begin(); pose_it != markers.end(); ++pose_it) {
		found.insert(pose_it->id);	// the ones we found
	}
	// now find the others
	for(list<MarkerInfo>::const_iterator pose_it = markers.begin(); pose_it != markers.end(); ++pose_it) {
		// *pose_it = c_T_m1
		list<int> neighbors;	// the ones we potentially can also find
		getNeighbors(pose_it->id, neighbors);
		for(list<int>::const_iterator neighbor_it = neighbors.begin(); neighbor_it != neighbors.end(); ++neighbor_it) {
			if(found.count(*neighbor_it) == 0) {	// we didnt find it already
				int id1 = pose_it->id;
				int id2 = *neighbor_it;

				Pose2d m1_T_m2;
				if(id1 < id2) {
					m1_T_m2 = mapping_relations_.at(id1).at(id2);	// this is actually m1_T_m2 already
				} else {
					m1_T_m2 = mapping_relations_.at(id2).at(id1);	// this is m2_T_m1 => invert
					m1_T_m2.invert();	// => make it m1_T_m2

				}
				Pose2d c_T_m1(pose_it->w_T_m);
				Pose2d c_T_m2(c_T_m1 * m1_T_m2);	// c_T_m2
				c_T_m2.id = id2;

				additional_poses.push_back(c_T_m2);
			}
		}
	}

	// merge duplicates as weighted mean of the poses with count serving again as weight
	if(additional_poses.size() > 1) {
		list<Pose2d>::const_iterator end1 = additional_poses.end();
		--end1;
		for(list<Pose2d>::iterator it1 = additional_poses.begin(); it1 != end1; ++it1) {
			list<Pose2d>::iterator it2 = it1;
			++it2;
			list<Pose2d> duplicates;
			for(; it2 != additional_poses.end();) {
				if(it1->id == it2->id) {
					duplicates.push_back(*it2);
					if(end1 == it2) {
						it2 = additional_poses.erase(it2);
						end1 = it2;
					} else {
						it2 = additional_poses.erase(it2);
					}
				} else {
					++it2;
				}
			}
			// we have duplicates => determine weighted mean
			if(!duplicates.empty()) {
				duplicates.push_front(*it1);
				vector<float> weights(duplicates.size());
				int i = 0;
				for(list<Pose2d>::const_iterator it = duplicates.begin(); it != duplicates.end(); ++it, ++i) {
					weights[i] = it->count;
				}
				*it1 = weightedMeanPose(duplicates, weights);
			}
		}
	}

	rois.reserve(additional_poses.size());
	// project the additional ones into image plane
	for(list<Pose2d>::const_iterator it = additional_poses.begin(); it != additional_poses.end(); ++it) {
		// if we get this far, this means the marker center point will be visible inside the cutoff radius => test if it is in our actual fov
		int marker_id = it->id;

		// marker in camera coordinates: c_T_m
		Pose2d c_T_m(*it);

		// the goal is to accept all that could theoretically be in there and check if this is actually the case
		vector<cv::Point3f> object_points;
		object_points.reserve(8);

		float marker_length = Config::tracker_parameters.special_marker_sizes_mm.count(marker_id) ? Config::tracker_parameters.special_marker_sizes_mm[marker_id] : Config::tracker_parameters.marker_size_mm;
		MarkerProjectedCorners *corners = projected_corners_by_size_[marker_length];
		// marker area including tolerance
		for(size_t i = 0; i < 8; ++i) {
			object_points.push_back(pointCoordinate(c_T_m, corners->points[i]));
		}

		vector<cv::Point2f> projected_points;
		// project the object points to the distorted image
		Undistorter::projectPoints(object_points, projected_points);
		// omit the roi if one of the two following conditions is met:
		// 1) more than 3 points are projected outside the image or very near the image border (within border margin or beyond)
		// 2) if two points are projected outside the image, the must not be too far out (see -border_margin)
		// reason: we get biased measurements and hence cannot expect a good result in this case
		MarkerROI marker_roi(marker_id);

		const float border_margin = 5.f;
		int points_outside_image = 0;
		for(size_t i = 4; i < 8; ++i) {
			float x_dist = min(projected_points[i].x, Config::camera_parameters.camera_size.width - 1 - projected_points[i].x);
			float y_dist = min(projected_points[i].y, Config::camera_parameters.camera_size.height - 1 - projected_points[i].y);
			bool outside_image = min(x_dist, y_dist) < border_margin;
			marker_roi.corner_inside_image[i-4] = !outside_image;
			points_outside_image += outside_image;
		}
		num_rois_completely_inside_ += points_outside_image == 0;

		// compute distance to all image borders and determine shortest one for each roi
		if(points_outside_image < 3 && getMarkerROI(projected_points, marker_roi, Config::camera_parameters.camera_size)) {
			//if(points_outside_image < 2 && getMarkerROI(projected_points, marker_roi, image_size_)) {
			// how long each side will be in the image
			cv::Point2f corners[4] = { projected_points[4], projected_points[5], projected_points[6], projected_points[7] };
			float fx_weight = computeFXWeight(corners, Config::calibration_parameters.c);
			float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1-fx_weight) * Config::calibration_parameters.f.y;
			float line_length = (marker_length * f_wmean)/marker_heights_.at(marker_id);

			if(min(marker_roi.roi.height, marker_roi.roi.width) >= line_length) {
				++num_rois_;
				// set min/max allowed line length to roi
				marker_roi.projected_line_length[0] = line_length;	// target
				marker_roi.projected_line_length[1] = FLOOR((1.0f - Config::tracker_parameters.marker_tolerance) * line_length); // min threshold
				marker_roi.projected_line_length[2] = CEIL((1.0f + Config::tracker_parameters.marker_tolerance) * line_length); // max threshold

				// add projected points for the original marker corners (places 4-8)
				marker_roi.corners.push_back(projected_points[4]);
				marker_roi.corners.push_back(projected_points[5]);
				marker_roi.corners.push_back(projected_points[6]);
				marker_roi.corners.push_back(projected_points[7]);
				rois.push_back(marker_roi);
			}
		}
	}
	// no sorting necessary, since we want all marker rois inside the image during mapping
}

/**
 * There is a lot of duplicated code in this function, but i dont have the time to unify them properly.
 *
 * @param c_T_m Poses of the found markers
 * @param rois return values: rois of further markers
 */
void MappingMarkerPredictor::getTrackedROIs(const list<MarkerInfo> &markers, vector<MarkerROI> &rois, const int timestamp) {
	rois.clear();

	set<int> found;
	list<Pose2d> additional_poses;	// c_T_x (in camera coordinates)

	for(list<MarkerInfo>::const_iterator pose_it = markers.begin(); pose_it != markers.end(); ++pose_it) {
		found.insert(pose_it->id);	// the ones we found
	}

	// find others to track
	for(map<int, VectorLocationPredictor>::const_iterator track_it = trackers_.begin(); track_it != trackers_.end(); ++track_it) {
		if(found.count(track_it->first) == 0) { // new marker
			additional_poses.push_back(trackers_.at(track_it->first).predict(timestamp));
		}
	}

	rois.reserve(additional_poses.size());
	// project the additional ones into image plane
	for(list<Pose2d>::const_iterator it = additional_poses.begin(); it != additional_poses.end(); ++it) {
		// if we get this far, this means the marker center point will be visible inside the cutoff radius => test if it is in our actual fov
		int marker_id = it->id;

		// marker in camera coordinates: c_T_m
		Pose2d c_T_m(*it);

		// the goal is to accept all that could theoretically be in there and check if this is actually the case
		vector<cv::Point3f> object_points;
		object_points.reserve(8);

		float marker_length = Config::tracker_parameters.special_marker_sizes_mm.count(marker_id) ? Config::tracker_parameters.special_marker_sizes_mm[marker_id] : Config::tracker_parameters.marker_size_mm;
		MarkerProjectedCorners *corners = projected_corners_by_size_[marker_length];
		// marker area including tolerance
		for(size_t i = 0; i < 8; ++i) {
			object_points.push_back(pointCoordinate(c_T_m, corners->points[i]));
		}

		vector<cv::Point2f> projected_points;
		// project the object points to the distorted image
		Undistorter::projectPoints(object_points, projected_points);
		// omit the roi if one of the two following conditions is met:
		// 1) more than 3 points are projected outside the image or very near the image border (within border margin or beyond)
		// 2) if two points are projected outside the image, the must not be too far out (see -border_margin)
		// reason: we get biased measurements and hence cannot expect a good result in this case
		MarkerROI marker_roi(marker_id);

		const float border_margin = 5.f;
		int points_outside_image = 0;
		for(size_t i = 4; i < 8; ++i) {
			float x_dist = min(projected_points[i].x, Config::camera_parameters.camera_size.width - 1 - projected_points[i].x);
			float y_dist = min(projected_points[i].y, Config::camera_parameters.camera_size.height - 1 - projected_points[i].y);
			bool outside_image = min(x_dist, y_dist) < border_margin;
			marker_roi.corner_inside_image[i-4] = !outside_image;
			points_outside_image += outside_image;
		}
		num_rois_completely_inside_ += points_outside_image == 0;

		// compute distance to all image borders and determine shortest one for each roi
		if(points_outside_image < 3 && getMarkerROI(projected_points, marker_roi, Config::camera_parameters.camera_size)) {
			//if(points_outside_image < 2 && getMarkerROI(projected_points, marker_roi, image_size_)) {
			// how long each side will be in the image
			cv::Point2f corners[4] = { projected_points[4], projected_points[5], projected_points[6], projected_points[7] };
			float fx_weight = computeFXWeight(corners, Config::calibration_parameters.c);
			float f_wmean = fx_weight * Config::calibration_parameters.f.x + (1-fx_weight) * Config::calibration_parameters.f.y;
			float line_length = (marker_length * f_wmean)/marker_heights_.at(marker_id);

			if(min(marker_roi.roi.height, marker_roi.roi.width) >= line_length) {
				++num_rois_;
				// set min/max allowed line length to roi
				marker_roi.projected_line_length[0] = line_length;	// target
				marker_roi.projected_line_length[1] = FLOOR((1.0f - Config::tracker_parameters.marker_tolerance) * line_length); // min threshold
				marker_roi.projected_line_length[2] = CEIL((1.0f + Config::tracker_parameters.marker_tolerance) * line_length); // max threshold

				// add projected points for the original marker corners (places 4-8)
				marker_roi.corners.push_back(projected_points[4]);
				marker_roi.corners.push_back(projected_points[5]);
				marker_roi.corners.push_back(projected_points[6]);
				marker_roi.corners.push_back(projected_points[7]);
				rois.push_back(marker_roi);
			}
		}
	}
}

}
