/*
 * main.cpp
 *
 *  Created on: 02.02.2015
 *      Author: jung
 */

#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/random.hpp>
#include <opencv2/opencv.hpp>
#include "autoadjust/matrixautoexposurealgorithm.h"
#include "autoadjust/exposurelimiter.h"
#include "configure/config.h"
#include "imgproc.h"
#include "net/posetopayloadtranslator.h"
#include "net/udppacketsender.h"
#include "marker/fullmarkerextractor.h"
#include "marker/newroimarkerextractor.h"
#include "marker/perfectmarkerpredictor.h"
#include "marker/mappingmarkerpredictor.h"
#include "location/coordinatetransformer.h"
#include "location/vectorlocationpredictor.h"
#include "location/kalmanlocationpredictor.h"
#include "location/pppestimator.h"
#include "location/pnpestimator.h"
#include "location/simpleposestorage.h"
#include "location/evaluationposestorage.h"
#include "generic/helper.h"
#include "imageprocessor.h"	// has to be at the end due to issues with including windows.h

using namespace std;
using namespace fml;

bool createRunFile() {
	fstream fs;
	fs.open("running.txt", fstream::out);
	bool success = fs.is_open();
	if(success) {
		fs << endl;
		fs.close();
	}
	return(success);
}

bool existsRunFile() {
	fstream fs;
	fs.open("running.txt", fstream::in);
	bool success = fs.is_open();
	if(success) {
		fs.close();
	}
	return(success);
}

void readConfig(string &folder, string &filename) {
	// config file containing the other config files for calibration, camera, tracker
	cv::FileStorage fs(string(folder).append(filename), cv::FileStorage::READ);

	// read configuration files
	string calibration_file, camera_file, marker_file, marker_file_gt, tracker_file;
	fs["calibration"] >> calibration_file;
	fs["camera"] >> camera_file;
	fs["marker"] >> marker_file;
	fs["marker_gt"] >> marker_file_gt;
	fs["tracker"] >> tracker_file;
	fs.release();

	// parameters read from the above filenames
	CalibrationParameters calibration_parameters = CalibrationParameters(string(folder).append(calibration_file));
	CameraParameters camera_parameters = CameraParameters(string(folder).append(camera_file));
	MarkerParameters marker_parameters = MarkerParameters(string(folder).append(marker_file));
	MarkerParameters marker_parameters_gt = MarkerParameters(string(folder).append(marker_file_gt));
	TrackerParameters tracker_parameters = TrackerParameters(string(folder).append(tracker_file));
	Config::setConfigs(calibration_parameters, camera_parameters, marker_parameters, marker_parameters_gt, tracker_parameters);
}

void compactMapData(const map<int, map<int, list<Pose2d>* const>* const> &map_data) {
	cv::Mat map_storage(1, 17, MAT_TYPE);    // 3 cols, 4 rows
	// first compress data (weighted mean), so we do not need to store every single pose we captured
	for(map<int, map<int, list<Pose2d>* const>* const>::const_iterator outer_it = map_data.begin(); outer_it != map_data.end(); ++outer_it) {
		for(map<int, list<Pose2d>* const>::const_iterator inner_it = outer_it->second->begin(); inner_it != outer_it->second->end(); ++inner_it) {
			size_t inner_size = inner_it->second->size();
			if(inner_size > 0) {
				inner_size = max(inner_size, (size_t)1);
				float stdev[4] = { 0 };
				// we use the mean as approximation of the expectancy
				Pose2d mean(meanPose(*(inner_it->second), stdev, true));
				Pose2d inv_mean(mean);	// inverted mean
				inv_mean.invert();
				Pose2d mean_dev(mean.id, mean.source_id, mean.timestamp, mean.error, stdev[0], stdev[1], stdev[2], stdev[3]);
				/*for(list<Pose2d>::const_iterator pose_it = inner_it->second->begin(); pose_it != inner_it->second->end(); ++pose_it) {
					cout << pose_it->translation3d[0] << "," << pose_it->translation3d[1] << "," << pose_it->rotation*g_180_PI << endl;
				}*/

				inner_size = inner_it->second->size();
				// compute covariances for the information matrix (we only consider the 2d case here)
				// because we have limited data, we have to approximate the expectancy by the mean value
				double covariances[6] = { 0 }; // dxdx, dxdy, dxdtheta, dydy, dydtheta, dthetadtheta
				for(list<Pose2d>::const_iterator pose_it = inner_it->second->begin(); pose_it != inner_it->second->end(); ++pose_it) {
					double dx = pose_it->translation3d[0] - mean.translation3d[0];
					double dy = pose_it->translation3d[1] - mean.translation3d[1];
					double dtheta = pose_it->rotation - mean.rotation;
					dtheta += (dtheta > g_PI) * -g_PI2;
					dtheta += (dtheta < -g_PI) * g_PI2;
					covariances[0] += dx * dx;
					covariances[1] += dx * dy;
					covariances[2] += dx * dtheta;
					covariances[3] += dy * dy;
					covariances[4] += dy * dtheta;
					covariances[5] += dtheta * dtheta;
				}
				double rnum_values = 1.0/inner_size;
				for(size_t i = 0; i < 6; ++i) {
					covariances[i] *= rnum_values;
				}

				// create covariance matrix from those values
				cv::Mat covariance_matrix(3, 3, CV_64F);
				double *covariance_row = (double*)covariance_matrix.data;
				covariance_row[0] = covariances[0];	// dxdx
				covariance_row[1] = covariances[1]; // dxdy
				covariance_row[3+0] = covariances[1];
				covariance_row[2] = covariances[2]; // dxdtheta
				covariance_row[6+0] = covariances[2];
				covariance_row[3+1] = covariances[3]; // dydy
				covariance_row[3+2] = covariances[4]; // dydtheta
				covariance_row[6+1] = covariances[4];
				covariance_row[6+2] = covariances[5]; // dthetadtheta

				// compute inverse of the covariance matrix, which is symmetric and positive definite => choleskey decomposition is the fastest
				cv::Mat inv_covariance_matrix(3, 3, CV_64F);
				invert(covariance_matrix, inv_covariance_matrix, cv::DECOMP_CHOLESKY);

				// cleanup
				inner_it->second->clear();
				inner_it->second->push_back(mean);
				inner_it->second->push_back(mean_dev);

				// set the values
				covariance_row = (double*)inv_covariance_matrix.data;
				cv::Mat map_row = cv::Mat(1, 17, MAT_TYPE);  // 3 cols
				float *row_ptr = (float*)map_row.data;
				row_ptr[0] = outer_it->first;	// source id
				row_ptr[1] = inner_it->first;	// target id

				row_ptr[2] = mean.translation3d[0];	// x
				row_ptr[3] = mean.translation3d[1];	// y
				row_ptr[4] = mean.translation3d[2];	// z
				row_ptr[5] = mean.rotation; // yaw
				row_ptr[6] = mean_dev.translation3d[0]; // x_stdev
				row_ptr[7] = mean_dev.translation3d[1]; // y_stdev
				row_ptr[8] = mean_dev.translation3d[2]; // z_stdev
				row_ptr[9] = mean_dev.rotation; // yaw_stdev
				row_ptr[10] = inner_size;	// number of used measurements
				row_ptr[11] = (float)covariance_row[0]; // dxdx
				row_ptr[12] = (float)covariance_row[1]; // dxdy
				row_ptr[13] = (float)covariance_row[2]; // dxdtheta
				row_ptr[14] = (float)covariance_row[3+1]; // dydy
				row_ptr[15] = (float)covariance_row[3+2]; // dydtheta
				row_ptr[16] = (float)covariance_row[6+2]; // dthetadtheta
				map_storage.push_back(map_row);

				// now add inverse
				row_ptr[0] = inner_it->first;	// source id
				row_ptr[1] = outer_it->first;	// target id
				row_ptr[2] = inv_mean.translation3d[0];	// x
				row_ptr[3] = inv_mean.translation3d[1];	// y
				row_ptr[4] = inv_mean.translation3d[2];	// z
				row_ptr[5] = inv_mean.rotation; // yaw
				row_ptr[6] = mean_dev.translation3d[0]; // x_stdev
				row_ptr[7] = mean_dev.translation3d[1]; // y_stdev
				row_ptr[8] = mean_dev.translation3d[2]; // z_stdev
				row_ptr[9] = mean_dev.rotation; // yaw_stdev
				row_ptr[10] = inner_size;	// number of used measurements
				row_ptr[11] = (float)covariance_row[0]; // dxdx
				row_ptr[12] = (float)covariance_row[1]; // dxdy
				row_ptr[13] = (float)covariance_row[2]; // dxdtheta
				row_ptr[14] = (float)covariance_row[3+1]; // dydy
				row_ptr[15] = (float)covariance_row[3+2]; // dydtheta
				row_ptr[16] = (float)covariance_row[6+2]; // dthetadtheta
				map_storage.push_back(map_row);

				// print em
				cout << outer_it->first << "->" << inner_it->first << ":" << endl;
				cout << mean.translation3d[0] << "," << mean.translation3d[1] << "," << mean.translation3d[2] << ",";
				cout << mean.rotation << endl;
				cout << stdev[0] << "," << stdev[1] << "," << stdev[2] << "," << stdev[3] << "," << inner_size << endl;
			}
		}
	}

	// crop first line
	cv::Rect roi(cv::Point(0,1), cv::Size(map_storage.cols, map_storage.rows-1));
	cv::Mat marker_relations(map_storage, roi);
	cv::FileStorage fs("map.xml", cv::FileStorage::WRITE);
	fs << "mapData" << marker_relations;
	fs.release();
}

set<int> findRowsForSourceId(const cv::Mat &matrix, int id) {
	set<int> retval;
	float* data = (float*)matrix.data;
	for(int i = 0; i < matrix.rows; ++i) {
		if(data[0] == id) {
			retval.insert(i);
		}
		data += matrix.cols;
	}
	return(retval);
}

set<int> findRowsForTargetId(const cv::Mat &matrix, const int id) {
	set<int> retval;
	float* data = (float*)matrix.data;
	for(int i = 0; i < matrix.rows; ++i) {
		if(data[1] == id) {
			retval.insert(i);
		}
		data += matrix.cols;
	}
	return(retval);
}

string create2DVertex(const string &prefix, const char separator, const int i, const float x, const float y, const float theta) {
	ostringstream temp;
	temp << prefix << separator << i << separator << x << separator << y << separator << theta << endl;
	return(temp.str());
}

string create2DEdgeG2o(const string &prefix, const char separator, const int i, const int j, const float d_x, const float d_y, const float d_theta, const float i_x, const float i_xy, const float i_xtheta, const float i_y, const float i_ytheta, const float i_theta) {
	ostringstream temp;
	temp << prefix << separator << i << separator << j << separator << d_x << separator << d_y << separator << d_theta << separator
			<< i_x << separator << i_xy << separator << i_xtheta << separator << i_y << separator << i_ytheta << separator << i_theta << endl;
	return(temp.str());
}

string create2DEdgeToro(const string &prefix, const char separator, const int i, const int j, const float d_x, const float d_y, const float d_theta, const float i_x, const float i_xy, const float i_xtheta, const float i_y, const float i_ytheta, const float i_theta) {
	ostringstream temp;
	temp << prefix << separator << i << separator << j << separator << d_x << separator << d_y << separator << d_theta << separator
			<< i_x << separator << i_xy << separator << i_y << separator << i_theta << separator << i_xtheta << separator << i_ytheta << endl;
	return(temp.str());
}

// cheap implementation of Dijkstra's Algorithm (without the priority queue). Not very efficient, but we only have 14 nodes in fml testarea, so this will do.
// Even if you were to use it on 1000 vertices, a more efficient implementation would not be feasible, since we only run this algorithm once during mapping.
// The result is optimal in the sense that the minimal distance (edge weights) are determined
void dijkstra(const int start_id, const cv::Mat &map_data, map<int, Pose2d> &poses) {
	// determine relevant ids
	set<int> targets;
	for(int i = 0; i < map_data.rows; ++i) {
		targets.insert((int)map_data.at<float>(i,0));
	}

	if(targets.count(start_id) == 0) {
		cerr << "mapping failed because we did not gather information on start id" << endl;
		return;
	}

	// for simplicity the start id is coordinate system origin
	poses.insert(pair<int, Pose2d>(start_id, Pose2d(start_id, 0, 0, 0, 0, 0, 0, 0)));

	// initialize
	list<DijkstraNode> Q;
	for(set<int>::const_iterator target_it = targets.begin(); target_it != targets.end(); ++target_it) {
		// add next node
		int id = *target_it;
		DijkstraNode node(id);
		if(id == start_id) {	// initialize startnode
			node.pred = node.id;
			node.distance = 0;
		}
		Q.push_back(node);
	}

	// dijkstra
	set<int> finished;
	while(!Q.empty()) {
		list<DijkstraNode>::iterator u = Q.begin();
		float u_dist = u->distance;
		list<DijkstraNode>::iterator q_it = Q.begin();
		++q_it;
		for(; q_it != Q.end(); ++q_it) {
			if(q_it->distance < u_dist) {
				u_dist = q_it->distance;
				u = q_it;
			}
		}
		// stop if distance for all is infinity (we use FLT_MAX to represent infinity - i know there are better ways to do that...)
		if(u_dist == FLT_MAX) { // all other nodes are not connected to start_id
			break;
		}

		// insert it into finished and erase from Q (and determine pose of course)
		int u_id = u->id;
		if(u_id != start_id) {	// the start_id is always 0,0,0,0 and already "determined" above
			set<int> candidate_rows = findRowsForTargetId(map_data, u_id);
			for(set<int>::const_iterator cand_it = candidate_rows.begin(); cand_it != candidate_rows.end(); ++cand_it) {
				int source_id = map_data.at<float>(*cand_it, 0);
				if(source_id == u->pred) {
					const float *row = map_data.ptr<float>(*cand_it);
					Pose2d source(poses.at(source_id));
					Pose2d edge(u_id, source_id, 0, 0, row[2], row[3], row[4], row[5]);
					Pose2d u_Pose(source * edge);
					u_Pose.error = u->distance;
					poses.insert(pair<int, Pose2d>(u_id, u_Pose));
					break;
				}
			}
		}
		finished.insert(u->id);
		Q.erase(u);

		// get neighbors and remove those we do not need to consider
		set<int> v_helper = findRowsForSourceId(map_data, u_id);	// rows with neighbor info
		set<int> v;	// neighbors
		map<int, float>	distance_uv;
		for(set<int>::const_iterator v_it = v_helper.begin(); v_it != v_helper.end(); ++v_it) {
			const float *row = map_data.ptr<float>(*v_it);
			int v_id = row[1];
			int num_measurements = row[10];
			// measurement is not good enough or minimal distance already determined
			if(num_measurements >= Config::tracker_parameters.mapping_minmeasurements && finished.count(v_id) == 0) {
				float distance = 1.0f/log10f(num_measurements);
				//float distance = SQRT(row[6]*row[6] + row[7]*row[7] + row[8]*row[8]);
				distance_uv.insert(pair<int,float>(v_id, distance));
				v.insert(v_id);
			}
		}
		// update the neighbors we will consider
		for(list<DijkstraNode>::iterator q_it = Q.begin(); q_it != Q.end(); ++q_it) {
			if(v.count(q_it->id) == 1) {
				float alternative = u_dist + distance_uv[q_it->id];
				// update if lower
				if(alternative < q_it->distance) {
					q_it->distance = alternative;
					q_it->pred = u_id;
				}
			}
		}
	}
}

// minimum spanning tree algorithm (prim)
void mst(const int start_id, const cv::Mat &map_data, map<int, Pose2d> &poses) {
	set<int> available;
	list<GraphEdge> edges;
	list<GraphEdge> mst_edges;

	// initialize
	for(int i = 0; i < map_data.rows; i+=2) {
		const float *row = map_data.ptr<float>(i);
		int src = row[0];
		int dst = row[1];
		float cost = log10f(row[10]);
		//float cost = SQRT(row[6]*row[6] + row[7]*row[7] + row[8]*row[8]);
		float x = row[2];
		float y = row[3];
		float z = row[4];
		float yaw = row[5];
		if(row[10] >= Config::tracker_parameters.mapping_minmeasurements) {
			available.insert(src);
			available.insert(dst);
			edges.push_back(GraphEdge(src, dst, cost, x, y, z, yaw));
		}
	}
	// sort edges by size
	edges.sort();
	edges.reverse();

	// start at start_id
	available.erase(start_id);
	poses.insert(pair<int, Pose2d>(start_id, Pose2d(start_id, start_id, 0, 0, 0, 0, 0, 0)));

	// main loop
	size_t previous_size = available.size();
	while(!available.empty()) {
		for(list<GraphEdge>::iterator edge_it = edges.begin(); edge_it != edges.end();) {
			if(available.count(edge_it->src) == 0 && available.count(edge_it->dst) == 1) {
				cerr << "added " << edge_it->src << "->" << edge_it->dst << endl;
				available.erase(edge_it->dst);
				Pose2d next_vertex(poses.at(edge_it->src) * edge_it->relation);
				poses.insert(pair<int, Pose2d>(edge_it->dst, next_vertex));
				edge_it = edges.erase(edge_it);
				break;
			} else if(available.count(edge_it->dst) == 0 && available.count(edge_it->src) == 1) {
				available.erase(edge_it->src);
				edge_it->relation.invert();
				cerr << "added " << edge_it->dst << "->" << edge_it->src << endl;
				Pose2d next_vertex(poses.at(edge_it->dst) * edge_it->relation);
				poses.insert(pair<int, Pose2d>(edge_it->src, next_vertex));
				edge_it = edges.erase(edge_it);
				break;
			} else if(available.count(edge_it->dst) == 0 && available.count(edge_it->src) == 0) {
				edge_it = edges.erase(edge_it);
			} else {
				++edge_it;
			}
		}
		if(available.size() == previous_size) {
			// no more connections
			break;
		} else {
			previous_size = available.size();
		}
	}
}

void createMap() {
	cv::FileStorage fs("map.xml", cv::FileStorage::READ);
	cv::Mat map_data;
	fs["mapData"] >> map_data;
	fs.release();
	assert(map_data.cols == 17);

	// create initial map as input for graph optimization
	const char separator = ' ';

	// poses
	map<int, Pose2d> poses;
	// minimum spanning tree (kruskal)
	//mst(tracker_parameters.mapping_startid, map_data, poses);
	dijkstra(Config::tracker_parameters.mapping_startid, map_data, poses);

	// now create input for graph optimization
	const string vertex_prefix_g2o("VERTEX_SE2");
	const string edge_prefix_g2o("EDGE_SE2");
	const string vertex_prefix_toro("VERTEX2");
	const string edge_prefix_toro("EDGE2");
	const string filename_g2o("input.g2o");
	const string filename_toro("input.toro");
	ofstream g2o_input, toro_input;
	g2o_input.open(filename_g2o.c_str(), ofstream::out);
	toro_input.open(filename_toro.c_str(), ofstream::out);

	if(g2o_input.is_open() && toro_input.is_open()) {
		// start id
		g2o_input << create2DVertex(vertex_prefix_g2o, separator, 0, 0, 0, 0);
		toro_input << create2DVertex(vertex_prefix_toro, separator, 0, 0, 0, 0);
		g2o_input << "FIX " << 0 << endl;
		toro_input << "FIX " << 0 << endl;

		cv::Mat marker_map(poses.size(), 5, MAT_TYPE);
		int i  = 0;
		// vertices
		for(map<int, Pose2d>::iterator it = poses.begin(); it != poses.end(); ++it) {
			int id = it->first;
			it->second.translation3d[2] = Config::marker_parameters.base_T_m.at(id).translation3d[2];
			float* marker_row = marker_map.ptr<float>(i);
			marker_row[0] = id;
			marker_row[1] = it->second.translation3d[0];
			marker_row[2] = it->second.translation3d[1];
			marker_row[3] = it->second.translation3d[2];
			marker_row[4] = it->second.rotation;
			++i;
			cout << "Marker Coordinates for id " << it->first << endl;
			it->second.print();

			if(id != Config::tracker_parameters.mapping_startid) {
				g2o_input << create2DVertex(vertex_prefix_g2o, separator, id, it->second.translation3d[0], it->second.translation3d[1], it->second.rotation);
				toro_input << create2DVertex(vertex_prefix_toro, separator, id, it->second.translation3d[0], it->second.translation3d[1], it->second.rotation);
			}
		}

		// edges
		for(int i = 0; i < map_data.rows; i += 2) {
			const float *map_row = map_data.ptr<float>(i);
			if(map_row[10] > Config::tracker_parameters.mapping_minmeasurements) {
				float source_id = map_row[0] == Config::tracker_parameters.mapping_startid ? 0 : map_row[0];
				float target_id = map_row[1] == Config::tracker_parameters.mapping_startid ? 0 : map_row[1];
				// information matrix for vertex
				float i_x = map_row[11];
				float i_xy = map_row[12];
				float i_xtheta = map_row[13];
				float i_y = map_row[14];
				float i_ytheta = map_row[15];
				float i_theta = map_row[16];
				/*i_x = 1;
				i_xy = 0;
				i_xtheta = 0;
				i_y = 1;
				i_ytheta = 0;
				i_theta = 1;*/

				g2o_input << create2DEdgeG2o(edge_prefix_g2o, separator, source_id, target_id, map_row[2], map_row[3], map_row[5], i_x, i_xy, i_xtheta, i_y, i_ytheta, i_theta);
				toro_input << create2DEdgeToro(edge_prefix_toro, separator, source_id, target_id, map_row[2], map_row[3], map_row[5], i_x, i_xy, i_xtheta, i_y, i_ytheta, i_theta);
			}
		}

		g2o_input.close();
		toro_input.close();

		cv::FileStorage fs2("marker_map.xml", cv::FileStorage::WRITE);
		fs2 << "Poses" << marker_map;
		fs2.release();
	} else {
		cerr << "Error: could not write to " << filename_g2o.c_str() << " or " << filename_toro.c_str() << "." << endl;
	}
}

// initialize some static stuff. this must under all circumstances run before anything else!
void setupStaticStuff() {
	ArucoMarkerDetector::init_static();
	ArucoMarkerIdentifier::init_static();
	PoseEstimator::init_static();
}

void cleanupStaticStuff() {
	ArucoMarkerIdentifier::clean_static();
}

int main(int argc, const char* const argv[]) {
	// some opencv settings regarding multithreading
	cv::setUseOptimized(false);
	cv::setNumThreads(0);

	string folder, filename;
	if(argc > 2) {
		folder = string(argv[1]);
		filename = string(argv[2]);
	} else {
		folder = string("config/");
		filename = string("config_3.xml");
	}

	createRunFile();	// create file indicating program is running

	// read configuration
	readConfig(folder, filename);

	if(Config::tracker_parameters.demo) {
		Config::tracker_parameters.evaluation = false;
		Config::tracker_parameters.mapping = false;
	}
	if(Config::tracker_parameters.mapping) {
		Config::tracker_parameters.evaluation = true;
		//Config::tracker_parameters.split_detection = 2;
		Config::tracker_parameters.marker_limit = std::numeric_limits<int>::max();
	}

	// initialize static datastructures first
	setupStaticStuff();

	cerr << "prediction: " << Config::tracker_parameters.prediction << endl;
	//createMap(); exit(0);
	//mainCorrectionPlanes(camera_parameters.z_offset); exit(0);

	// initialize random number generator
	srand(currentTimeOfDayMilliseconds());

	// marker extractors
	FullMarkerExtractor extractor_full;
	NewRoiMarkerExtractor extractor_rois;
	extractor_rois.setMaxMarker(Config::tracker_parameters.marker_limit);

	// marker corner point undistortion (if configured)
	//Undistorter undistorter(calibration_parameters.camera_matrix, calibration_parameters.distortion_coefficients);
	// pose estimation
	PoseEstimator *pose_estimator = NULL;
	switch(Config::tracker_parameters.pose_estimator) {
	case PoseEstimationAlgorithm::POSE_PPP:
		// jung's algorithm (based on bigontina's approach)
		// set last boolean parameter to true if you want to apply a corner correction method
		pose_estimator = new PPPEstimator();
		break;
	case PoseEstimationAlgorithm::POSE_ITERATIVE:
		cerr << "using ITERATIVE" << endl;
		pose_estimator = new PnPEstimator(false, CV_ITERATIVE);
		break;
	case PoseEstimationAlgorithm::POSE_EPNP:
		pose_estimator = new PnPEstimator(false, CV_EPNP);
		break;
	case PoseEstimationAlgorithm::POSE_P3P:
		pose_estimator = new PnPEstimator(false, CV_P3P);
		break;
	case PoseEstimationAlgorithm::POSE_POSIT:
		// not implemented (integrate here if interested, needs non-coplanar points, hence not useful for marker pose estimation)
		break;
	case PoseEstimationAlgorithm::POSE_PLANARPOSIT:
		// not implemented due to time constraints, can handle coplaner points though.
		break;
	default:	// use PPP
		pose_estimator = new PPPEstimator();
		break;
	}

	// coordinate transformer
	CoordinateTransformer transformer;

	// Predict vehicle location
	LocationPredictor *location_predictor = NULL;
	switch(Config::tracker_parameters.tracker_algorithm) {
	case TrackerAlgorithm::TRACK_VECTOR:
		location_predictor = new VectorLocationPredictor();
		break;
	case TrackerAlgorithm::TRACK_KALMAN:
		location_predictor = new KalmanLocationPredictor(Config::tracker_parameters.kalman_Rk_x, Config::tracker_parameters.kalman_Rk_theta, Config::tracker_parameters.kalman_Qk_x, Config::tracker_parameters.kalman_Qk_theta);
		break;
	default:	// use Vector
		location_predictor = new VectorLocationPredictor();
		break;
	}

	// Determine visible markers at predicted location
	PerfectMarkerPredictor roi_predictor;
	// same for mapping
	MappingMarkerPredictor mapping_predictor;

	// Write determined w_T_c to file
	SimplePoseStorage pose_storage1(string("pose_camera.csv"), Config::tracker_parameters.evaluation);	// store test results
	SimplePoseStorage pose_storage2(string("pose_vehicle.csv"), Config::tracker_parameters.evaluation);	// Vehicle coordinates
	SimplePoseStorage pose_storage3(string("predict_camera.csv"), Config::tracker_parameters.evaluation);	// predicted camera coordinates
	EvaluationPoseStorage pose_storage4(string("difference_predict_mean.csv"), Config::tracker_parameters.evaluation);	// difference between predicted and camera coordinates
	EvaluationPoseStorage pose_storage5(string("difference_predict_all.csv"), Config::tracker_parameters.evaluation);	// difference between predicted and marker coordinates
	EvaluationPoseStorage pose_storage6(string("difference_real_all.csv"), Config::tracker_parameters.evaluation);	// difference between marker pair relation measurement and marker pair ground truth
	EvaluationPoseStorage pose_storage7(string("difference_real_mean.csv"), Config::tracker_parameters.evaluation);	// same as difference_real_measured but mean per frame (which represents our mean_camera_pose more accurately)

	// difference between measured marker relations and actual relations
	map<int, list<MarkerRelation2D>* const> marker_relations;
	if(Config::tracker_parameters.evaluation) {
		for(map<int, Pose2d>::const_iterator it1 = Config::marker_parameters_gt.base_T_m.begin(); it1 != Config::marker_parameters_gt.base_T_m.end(); ++it1) {
			for(map<int, Pose2d>::const_iterator it2 = Config::marker_parameters_gt.base_T_m.begin(); it2 != Config::marker_parameters_gt.base_T_m.end(); ++it2) {
				if(it1->first < it2->first) {
					Pose2d m1_T_w(it1->second);
					m1_T_w.invert();
					Pose2d w_T_m2(it2->second);
					Pose2d m1_T_m2(m1_T_w * w_T_m2);
					if(marker_relations.count(it1->first) == 0) {
						marker_relations.insert(pair<int, list<MarkerRelation2D>* const>(it1->first, new list<MarkerRelation2D>()));
					}
					marker_relations.at(it1->first)->push_back(MarkerRelation2D(0, it1->first, it2->first, m1_T_m2.translation3d[0], m1_T_m2.translation3d[1], m1_T_m2.rotation, m1_T_m2.error));
				}
			}
		}
	}

	// send udp packets with localization contents
	PoseToPayloadTranslator translator;
	UDPPacketSender sender(Config::tracker_parameters.network_ip, Config::tracker_parameters.network_port);
	// benchmark
	ofstream benchmark_runtime("benchmark.csv");
	ofstream exposure_runtime("exposure.csv");
	ostringstream line_buffer, line_buffer2;
	line_buffer.str().reserve(20000);
	line_buffer2.str().reserve(10000);

	// calibrated image center
	cv::Point2f image_center(Config::calibration_parameters.camera_matrix.at<float>(0,2), Config::calibration_parameters.camera_matrix.at<float>(1,2));
	// norm factor for the weights of each pose in the joint pose
	float norm_factor_global_weight = 2.0f/(Config::calibration_parameters.camera_size.height + Config::calibration_parameters.camera_size.width);

	size_t num_poses = 0;
	size_t num_erased_poses = 0;
	size_t total_no_marker_found = 0;
	size_t total_predict_success = 0;
	size_t total_predict_failure = 0;
	bool run_at_least_once = true;
	const size_t ae_every_nth = 2;
	size_t num_experiments = 0;
	while(!Config::camera_parameters.input_str.empty() || run_at_least_once) {
		++num_experiments;
		run_at_least_once = false;

		// image processing stuff
		double initial_exposure = 500000.0f/Config::camera_parameters.fps;
		ImageProcessor image_processor;
		image_processor.setExposure(initial_exposure);

		bool prediction_possible = false;
		Pose2d previous;

		// for mapping and evaluation:
		list<MarkerRelation2D> map_update_list;

		size_t num_frames = 1;
		bool wait_fps = Config::camera_parameters.fps > 0;
		int ms_per_frame;
		size_t check_stop_file_every;
		if(wait_fps) {
			ms_per_frame = 1000/Config::camera_parameters.fps;
			check_stop_file_every = 10 * Config::camera_parameters.fps;
		} else {
			ms_per_frame = 1000/15;
			check_stop_file_every = 10 * 15;
		}

		bool run = true;
		bool attempt = true;
		int ms_account_balance = 0;

		int fail_count = 0;
		if(!Config::tracker_parameters.demo) {
			line_buffer << "#frame,grab,preprocessing,detection,identification,undistortion,estimation,pose,total,wait" << endl;
			line_buffer2 << "#frame,timestamp,exposure,exposure_limit," << endl;
		}
		size_t predict_success = 0;
		size_t predict_failure = 0;
		size_t no_marker_found = 0;

		MatrixAutoExposureAlgorithm ae(1000.0f, 1000000.0f/Config::camera_parameters.fps);
		ae.setValue(initial_exposure);
		// in case of live image first find decent exposure time
		if(image_processor.isLive()) {
			cv::Mat tmp;
			list<cv::Rect> exposure_rois;
			for(size_t i = 0; i < 10; ++i) {
				double start_time = (double)cv::getTickCount() * Config::time_ms_factor;
				if(image_processor.getRawImage(tmp) >= 0 && num_frames % ae_every_nth == 1) {
					if(ae.run(tmp, exposure_rois)) {
						image_processor.setExposure(ae.getValue());
					}
				}
				double finish_time = (double)cv::getTickCount() * Config::time_ms_factor;
				clock_t sleep_ms = ms_per_frame + start_time - finish_time;
				if(sleep_ms > 0) {
					sleepMilliSeconds(sleep_ms);
				}
			}
		}

		ExposureLimiter exposure_limiter;

		// now the main loop
		int eval_timestamp = 0;
		while(run) {
			bool run_ae = image_processor.isLive() && (num_frames % ae_every_nth == 1);
			double start_time = (double)cv::getTickCount() * Config::time_ms_factor;

			double after_grab = start_time, after_preprocessing = start_time, after_detect = start_time, after_identify = start_time, after_undistort = start_time, after_estimate = start_time;
			// obtain image and its timestamp
			cv::Mat raw_image;
			int image_timestamp = image_processor.getRawImage(raw_image);
			//flip(raw_image, raw_image, 0);

			Pose2d next_camera_pose;
			if(image_timestamp > 0) {	// we got an image => proceed
				//cerr << "frame: " << num_frames << "," << image_timestamp << endl;
				fail_count = 0;	// reset fail_count

				if(Config::tracker_parameters.evaluation && image_timestamp == INT_MAX) { // if we evaluate and didnt record timestamps (naughty)
					image_timestamp = eval_timestamp;
				}
				after_grab = (double)cv::getTickCount() * Config::time_ms_factor;

				vector<MarkerROI> rois;
				list<cv::Rect> exposure_rois;
				int skipped_ms = 0;
				if(Config::tracker_parameters.prediction && prediction_possible && !Config::tracker_parameters.mapping) {
					// predict next camera pose
					skipped_ms = image_timestamp - previous.timestamp;
					next_camera_pose = location_predictor->predict(previous, image_timestamp);
					pose_storage3.add(next_camera_pose);
					roi_predictor.predict(next_camera_pose, rois);

					for(size_t i = 0; i < rois.size(); ++i) {
						exposure_rois.push_back(rois[i].roi);
					}
				}
				if(run_ae) {
					line_buffer2 << num_frames << "," << image_timestamp << "," << ae.getValue() << "," << ae.getMax() << endl;
					if(ae.run(raw_image, exposure_rois)) {
						// change exposure
						image_processor.setExposure(ae.getValue());
						//cout << "set exposure to " << ae.getValue() << endl;
					}
				}

				float skip_factor = (skipped_ms > 0) ? (float)ms_per_frame/(float)skipped_ms : 1;
				list<MarkerInfo> markers;
				extractor_rois.num_frames = image_timestamp;
				markers = extractor_rois.getMarkers(raw_image, rois, after_preprocessing, after_detect, after_identify);
				bool roi_success = !markers.empty();
				/*cv::Mat raw_copy = raw_image.clone();
				for(size_t i = 0; i < rois.size(); ++i) {
					rectangle(raw_copy, rois[i].roi, cv::Scalar(128), 2);
				}
				putText(raw_copy, intToString(num_frames), cv::Size(100,100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2);
				imshow("", raw_copy);
				cv::waitKey();*/

				// count the corners in the image
				map<int, int> id_count;
				for(size_t i = 0; i < rois.size(); ++i) {
					int count = (int)rois[i].corner_inside_image[0] + (int)rois[i].corner_inside_image[1] + (int)rois[i].corner_inside_image[2] + (int)rois[i].corner_inside_image[3];
					id_count.insert(pair<int, int>(rois[i].id, count));
				}

				// if this fails as well, scan the entire frame
				if(markers.empty()) {
					++predict_failure;
					if(Config::tracker_parameters.prediction && !Config::tracker_parameters.mapping) {
						if(prediction_possible) {
							cerr << "prediction failed for frame ";
							/*cv::Mat raw_copy = raw_image.clone();
							for(size_t i = 0; i < rois.size(); ++i) {
								rectangle(raw_copy, rois[i].roi, cv::Scalar(128), 2);
							}
							string filename(intToString(num_frames).append(".bmp"));
							imwrite(filename, raw_copy);*/
						} else {
							cerr << "no prediction performed for frame ";
						}
						cerr << num_frames << "," << image_timestamp << endl;
					}

					markers = extractor_full.getMarkers(raw_image, after_preprocessing, after_detect, after_identify);

					// count the corners in the image
					for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
						id_count.insert(pair<int, int>(it->id, 4));
					}
				} else {
					++predict_success;
					after_preprocessing = (double)cv::getTickCount() * Config::time_ms_factor;
					after_detect = after_preprocessing;
					after_identify = after_preprocessing;
				}

				if(!markers.empty()) {
					// remove the markers we don't know about
					for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end();) {
						if(Config::marker_parameters.base_T_m.count(it->id)) {
							++it;
						} else {
							it = markers.erase(it);
						}
					}

					// DEBUG: draw marker corners
					/*cv::Mat raw_copy = raw_image.clone();
					cv::Mat raw_color;
					cvtColor(raw_copy, raw_color, CV_GRAY2BGR);
					vector<cv::Point2f> centers;
					centers.reserve(markers.size());
					for(list<MarkerInfo>::const_iterator it = markers.begin(); it != markers.end(); ++it) {
						if(marker_parameters.base_T_m.count(it->id)) {
							vector<cv::Point2f> vertices(it->vertices, it->vertices+4);
							centers.push_back(computeCenter(it->vertices));
							Undistorter::distortPoints(vertices);

							for(size_t i = 0; i < 4; ++i) {
								line(raw_color, vertices[i], vertices[MOD4((i+1))], cv::Scalar(0,255,0), 2);
							}
						}
					}*/
					/*Undistorter::distortPoints(centers);
					for(size_t i = 0; i < centers.size(); ++i) {
						for(size_t j = 1; j < centers.size(); ++j) {
							line(raw_color, centers[i], centers[j], cv::Scalar(0,0,255), 1);
						}
					}*/
					/*imshow("raw_color", raw_color);
					char retval = cv::waitKey();
					if(retval == 's') {*/
					/*const string zeros[6] = { "00000", "0000", "000", "00", "0", "" };
						string number(intToString(num_frames));
						string pre_zeros(zeros[number.size()]);
						string filename = pre_zeros.append(number).append(".png");
						imwrite(filename, raw_color);*/
					//}
				}

				after_undistort = (double)cv::getTickCount() * Config::time_ms_factor;
				// this estimator gives us the c_T_m for all markers
				if(roi_success && Config::tracker_parameters.pose_estimator == PoseEstimationAlgorithm::POSE_PPP && !Config::tracker_parameters.mapping) {
					((PPPEstimator*)pose_estimator)->estimatePose(markers, next_camera_pose);
				} else {
					pose_estimator->estimatePose(markers);	// estimate pose completely
				}
				after_estimate = (double)cv::getTickCount() * Config::time_ms_factor;

				// erase the worst measurements
				size_t erased_markers = 0;
				if(markers.size() > 1) {
					const float max_allowed_angle_deviation = g_PI/180.0f;	// deviation per frame from predicted angle > 1 degree
					set<int> erased;
					for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end();) {
						if(it->w_T_m.error > Config::tracker_parameters.max_error || skip_factor * it->w_T_m.dev > max_allowed_angle_deviation) {
							++erased_markers;
							cerr << "erased," << num_frames << "," << image_timestamp << "," << it->id << "," << it->w_T_m.error << "," << it->w_T_m.dev << endl;
							erased.insert(it->id);
							it = markers.erase(it);
						} else {
							++it;
						}
					}
				}
				num_erased_poses += erased_markers;

				// mapping: get additional measurements through marker projection
				if(Config::tracker_parameters.mapping && Config::tracker_parameters.prediction) {
					/*cv::Mat raw_copy;
					cvtColor(raw_image, raw_copy, CV_GRAY2BGR);
					for(list<MarkerInfo>::const_iterator it = markers.begin(); it != markers.end(); ++it) {
						vector<cv::Point2f> distorted(it->vertices, it->vertices+4);
						Undistorter::distortPoints(distorted);
						for(size_t i = 0; i < 4; ++i) {
							int next = MOD4((i+1));
							line(raw_copy, distorted[i], distorted[next], cv::Scalar(0,255,0), 2);
						}
					}*/
					// step 1: use tracking
					vector<MarkerROI> rois;
					mapping_predictor.getTrackedROIs(markers, rois, image_timestamp);
					/*for(size_t i = 0; i < rois.size(); ++i) {
						rectangle(raw_copy, rois[i].roi, cv::Scalar(255,0,0), 2);
					}*/
					if(!rois.empty()) {
						// determine new markers
						list<MarkerInfo> markers2 = extractor_rois.getMarkers(raw_image, rois, after_preprocessing, after_detect, after_identify);
						for(size_t i = 0; i < rois.size(); ++i) {
							int count = (int)rois[i].corner_inside_image[0] + (int)rois[i].corner_inside_image[1] + (int)rois[i].corner_inside_image[2] + (int)rois[i].corner_inside_image[3];
							id_count.insert(pair<int, int>(rois[i].id, count));
						}
						/*cout << image_timestamp << " found markers,";
						for(list<MarkerInfo>::const_iterator it = markers2.begin(); it != markers2.end(); ++it) {
							cout << it->id << ",";
						}
						cout << endl;*/
						pose_estimator->estimatePose(markers2);	// estimate pose completely
						markers.splice(markers.end(), markers2);
					}

					// step 2: for those markers found
					if(!markers.empty()) {
						mapping_predictor.getROIs(markers, rois);
						/*for(size_t i = 0; i < rois.size(); ++i) {
							rectangle(raw_copy, rois[i].roi, cv::Scalar(0,0,255), 2);
						}*/

						if(!rois.empty()) {
							// determine new markers
							list<MarkerInfo> markers2 = extractor_rois.getMarkers(raw_image, rois, after_preprocessing, after_detect, after_identify);
							for(size_t i = 0; i < rois.size(); ++i) {
								int count = (int)rois[i].corner_inside_image[0] + (int)rois[i].corner_inside_image[1] + (int)rois[i].corner_inside_image[2] + (int)rois[i].corner_inside_image[3];
								id_count.insert(pair<int, int>(rois[i].id, count));
							}
							//cout << "candidates,markers," << rois.size() << "," << markers2.size() << endl;
							pose_estimator->estimatePose(markers2);	// estimate pose completely
							/*int m = markers.size(), n = markers2.size();
							int standard = ((m*(m-1))>>1);
							int additional = n == 0 ? 0 : m*n + ((n*(n-1))>>1);
							cout << "relations," << standard << "," << additional << endl;*/
							markers.splice(markers.end(), markers2);
						} /*else {
							int m = markers.size();
							int standard = ((m*(m-1))>>1);
							cout << "relations," << standard << ",0" << endl;
						}*/
					}

					/*putText(raw_copy, intToString(markers.size()), cv::Size(100,100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2);
					imshow("markers", raw_copy);
					cv::waitKey();*/
					// update data structure and trackers
					mapping_predictor.updateTrackers(markers, image_timestamp);
					mapping_predictor.addRelations(markers);
					// determine which markers need our attention
					if(num_frames % 30 == 0) {
						list<int> missing = mapping_predictor.getMarkersNotConnected();
						if(!missing.empty()) {
							cout << "missing,";
							for(list<int>::const_iterator it = missing.begin(); it != missing.end(); ++it) {
								cout << *it << ",";
							}
							cout << endl;
						} else {
							cout << "done" << endl;
						}
					}
				}

				if(markers.empty()) {
					++no_marker_found;
					cerr << "no marker in frame " << num_frames << endl;
				}

				// update timestamp and transform to world coordinates
				num_poses += markers.size();
				Pose2d mean_camera_pose_copy;
				bool prediction_possible_copy = prediction_possible;
				if(!Config::tracker_parameters.mapping) {
					// camera pose in world
					list<Pose2d> camera_in_world_poses;
					// weighted average
					vector<float> weights;
					weights.reserve(markers.size());
					bool precondition = !Config::tracker_parameters.evaluation || Config::tracker_parameters.pos_uids.empty() || Config::tracker_parameters.marker_limit <= 10;
					for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
						if((precondition || Config::tracker_parameters.pos_uids.count(it->id) == 1) && transformer.toWorld(it->w_T_m)) {
							cv::Point2f marker_distance(computeCenter(it->vertices) - image_center);
							float weight = norm_factor_global_weight * (FABS(marker_distance.x) + FABS(marker_distance.y));
							weights.push_back(1-weight);
							camera_in_world_poses.push_back(it->w_T_m);
						}
					}

					// merge poses into one global camera pose in world
					if(!camera_in_world_poses.empty()) {
						Pose2d mean_camera_pose(weightedMeanPose(camera_in_world_poses, weights));

						if(roi_success && Config::tracker_parameters.pose_estimator == PoseEstimationAlgorithm::POSE_PPP) {
							bool precondition = !Config::tracker_parameters.evaluation || Config::tracker_parameters.pos_uids.empty() || Config::tracker_parameters.marker_limit <= 10;
							for(size_t i = 0; i < 2; ++i) {
								camera_in_world_poses.clear();
								((PPPEstimator*)pose_estimator)->estimatePose(markers, mean_camera_pose);
								for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
									if((precondition || Config::tracker_parameters.pos_uids.count(it->id) == 1) && transformer.toWorld(it->w_T_m)) {
										camera_in_world_poses.push_back(it->w_T_m);
									}
								}

								// now recompute the mean pose with the updated parts
								mean_camera_pose = weightedMeanPose(camera_in_world_poses, weights);
							}
						}
						mean_camera_pose.timestamp = image_timestamp;

						/*float worst_weight = weights[0];
						size_t worst_index = 0;
						for(size_t i = 1; i < weights.size(); ++i) {
							if(weights[i] < worst_weight) {
								worst_weight = weights[i];
								worst_index = i;
							}
						}
						list<Pose2d>::iterator it = camera_in_world_poses.begin();
						std::advance(it, worst_index);
						Pose2d mean_camera_pose(*it);*/
						if(prediction_possible) {	// for kalman filter
							mean_camera_pose = location_predictor->update(mean_camera_pose);
							exposure_limiter.setVelocity(location_predictor->getVelocityMSec(), location_predictor->getAngularVelocity());
						} else {
							exposure_limiter.addPose(mean_camera_pose);	// keep exposure limiter up to date if no predictor present of predictor not functional
						}
						pose_storage1.add(mean_camera_pose);

						// limit shutter speed based on velocity (translational or angular)
						list<int> ids;
						float max_distance = 0;
						for(size_t i = 0; i < rois.size(); ++i) {
							ids.push_back(rois[i].id);
							cv::Point2f predicted_corners[4] = { rois[i].corners[0], rois[i].corners[1], rois[i].corners[2], rois[i].corners[3] };
							cv::Point2f marker_center(computeCenter(predicted_corners));
							float dist = squared_distance(image_center, marker_center);
							if(dist > max_distance) {
								max_distance = dist;
							}
						}
						float exposure_limit = exposure_limiter.computeMaxExposure(ids, SQRT(max_distance));
						ae.setMax(exposure_limit);
						//cout << "set exposure limit to " << ae.getValue() << endl;
						if(run_ae && ae.getValue() > exposure_limit) {
							ae.setValue(exposure_limit);
							image_processor.setExposure(exposure_limit);
						}

						float v_trans = exposure_limiter.getSpeedMSec();
						float a_trans = exposure_limiter.getAccelMSec2();
						float v_rot = exposure_limiter.getAngularVelocity();
						float a_rot = exposure_limiter.getAccelRadSec2();

						prediction_possible = !camera_in_world_poses.empty();
						mean_camera_pose_copy = mean_camera_pose;
						previous = mean_camera_pose;
						//attempt = true; // TODO: check if we can set it here

						// finally transform camera pose to vehicle pose (which is the real information we are interested in)
						transformer.toVehicle(mean_camera_pose);
						mean_camera_pose.timestamp = image_timestamp;
						mean_camera_pose.translation3d[0] *= -1; // change x-axis
						pose_storage2.add(mean_camera_pose);

						if(Config::tracker_parameters.network) {
							/*const float full_circle = 6.0f;	// max m/s
							const float radius = 200.0f;
							// drawing of the indicator
							float v_trans_copy = v_trans;
							filter.smooth(v_trans_copy);
							cv::Mat speedometer_copy = speedometer.clone();
							cv::Point center(speedometer_copy.cols >> 1, speedometer_copy.rows >> 1);

							float part = min(fabs(v_trans_copy)/full_circle, 5.5f/6.0f);
							float angle = g_PI2 * (part > 0.5f ? part - (float)(1.5+1.0/full_circle) : part - (float)(0.5+1.0/full_circle));
							array<float, 2> pos = toPolar(angle);
							cv::Point target(radius*pos[0], radius*pos[1]);
							target += center;
							line(speedometer_copy, center, target, cv::Scalar(255,255,255), 2);

							const string zeros[6] = { "00000", "0000", "000", "00", "0", "" };
							string number = intToString(num_frames);
							string pre_zeros(zeros[number.size()]);
							string filename = string("speed_").append(pre_zeros.append(number).append(".png"));
							imwrite(filename, speedometer_copy);*/

							vector<char> payload;
							translator.translate(mean_camera_pose, payload, v_trans, a_trans, v_rot, a_rot);
							sender.send(payload);
						}
					} else {
						cout << exposure_limiter.getPixelSpeed() << endl;
						mean_camera_pose_copy = next_camera_pose;
						if(Config::tracker_parameters.prediction && attempt) {
							previous = next_camera_pose;
							Pose2d estimated_camera_pose(next_camera_pose);
							transformer.toVehicle(estimated_camera_pose);
							estimated_camera_pose.timestamp = image_timestamp;
							estimated_camera_pose.translation3d[0] *= -1;	// change x-axis
							pose_storage2.add(estimated_camera_pose);
							attempt = false;
						} else {
							location_predictor->reset();
							ae.reset();
							//cout << "reset exposure to " << ae.getValue() << endl;
							exposure_limiter.reset();
							prediction_possible = false;
							attempt = true;
						}
					}
				}

				if(!Config::tracker_parameters.demo) {
					if(!Config::tracker_parameters.mapping) {
						for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
							if(Config::tracker_parameters.pos_uids.count(it->id) == 0) {
								transformer.toWorld(it->w_T_m);
							}
						}
						if(Config::tracker_parameters.evaluation && prediction_possible_copy) {
							// compute deviation from predicted pose as a measure of accuracy
							Pose2d diff_pose;
							diff_pose.timestamp = skipped_ms;
							for(list<MarkerInfo>::const_iterator it = markers.begin(); it != markers.end(); ++it) {
								diff_pose.id = it->id;
								diff_pose.source_id = image_timestamp;
								for(size_t i = 0; i < 3; ++i) {
									diff_pose.translation3d[i] = (it->w_T_m.translation3d[i] - next_camera_pose.translation3d[i]) * skip_factor;
								}
								diff_pose.rotation = it->w_T_m.rotation - next_camera_pose.rotation;
								diff_pose.rotation += (diff_pose.rotation > g_PI) * -g_PI2;
								diff_pose.rotation += (diff_pose.rotation < -g_PI) * g_PI2;
								diff_pose.rotation *= skip_factor;
								diff_pose.count = id_count.at(it->id);
								diff_pose.error = it->w_T_m.error;
								pose_storage5.add(diff_pose);
							}

							diff_pose.id = -1;
							diff_pose.error = 0;
							diff_pose.count = markers.size();
							for(size_t i = 0; i < 3; ++i) {
								diff_pose.translation3d[i] = (mean_camera_pose_copy.translation3d[i] - next_camera_pose.translation3d[i]) * skip_factor;
							}
							diff_pose.rotation = mean_camera_pose_copy.rotation - next_camera_pose.rotation;
							diff_pose.rotation += (diff_pose.rotation > g_PI) * -g_PI2;
							diff_pose.rotation += (diff_pose.rotation < -g_PI) * g_PI2;
							diff_pose.rotation *= skip_factor;
							pose_storage4.add(diff_pose);
						}

						// now we need to go back to camera view for the marker relations
						for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end(); ++it) {
							it->w_T_m.invert(); // get c_T_w
							it->w_T_m = it->w_T_m * Config::marker_parameters.base_T_m.at(it->id);	// determine c_T_m = c_T_w * w_T_m
						}
					}

					// determine all marker relations (for map creation, updating and accuracy evaluation)
					if(markers.size() > 1) {
						if(Config::tracker_parameters.mapping) {
							// during mapping erase markers which are only partially inside the image
							for(list<MarkerInfo>::iterator it = markers.begin(); it != markers.end();) {
								bool keep = true;
								for(int i = 0; i < 4; ++i) {
									keep = keep && (it->vertices[i].x >= 0 && it->vertices[i].x < Config::camera_parameters.camera_size.width && it->vertices[i].y >= 0 && it->vertices[i].y < Config::camera_parameters.camera_size.height);
								}
								if(keep) {
									++it;
								} else {
									it = markers.erase(it);
								}
							}
						}
						list<MarkerInfo>::iterator end1 = markers.end();
						--end1;
						list<Pose2d> diff_poses;
						for(list<MarkerInfo>::const_iterator it = markers.begin(); it != end1; ++it) {
							list<MarkerInfo>::const_iterator it2 = it;
							++it2;
							for(; it2 != markers.end(); ++it2) {
								list<MarkerInfo>::const_iterator temp_it1, temp_it2;
								if(it->id < it2->id) {
									temp_it1 = it;
									temp_it2 = it2;
								} else {
									temp_it1 = it2;
									temp_it2 = it;
								}
								// insert pose
								Pose2d c_T_m1(temp_it1->w_T_m);
								c_T_m1.invert();	// now m1_T_c
								Pose2d c_T_m2(temp_it2->w_T_m);
								Pose2d m1_T_m2(c_T_m1 * c_T_m2);
								int count = id_count.at(it->id)*10 + id_count.at(it2->id);
								float error = SQRT(c_T_m1.error*c_T_m1.error+c_T_m2.error*c_T_m2.error);
								map_update_list.push_back(MarkerRelation2D(image_timestamp, temp_it1->id, temp_it2->id, m1_T_m2.translation3d[0], m1_T_m2.translation3d[1], m1_T_m2.rotation, error, count));

								int src_id = temp_it1->id;
								if(!Config::tracker_parameters.mapping && marker_relations.count(src_id)) {
									int dst_id = temp_it2->id;
									// mean difference
									list<MarkerRelation2D>* const inner_list = marker_relations.at(src_id);
									for(list<MarkerRelation2D>::const_iterator inner_it = inner_list->begin(); inner_it != inner_list->end(); ++inner_it) {
										if(inner_it->dst_id == dst_id) {
											float x = (m1_T_m2.translation3d[0] - inner_it->x);
											float y = (m1_T_m2.translation3d[1] - inner_it->y);
											float yaw = m1_T_m2.rotation - inner_it->angle;
											yaw += (yaw > g_PI) * -g_PI2;
											yaw += (yaw < -g_PI) * g_PI2;

											Pose2d diff(dst_id, src_id, image_timestamp, error, x, y, 0, yaw);
											diff.count = count;
											diff_poses.push_back(diff);
											break;
										}
									}
								}
							}
						}
						// fill mean diff pose (one mean value per frame)
						if(!diff_poses.empty()) {
							float stdev[4];
							Pose2d mean_diff_pose = meanPose(diff_poses, stdev);
							mean_diff_pose.id = -1;
							mean_diff_pose.source_id = -1;
							mean_diff_pose.count = diff_poses.size();
							pose_storage7.add(mean_diff_pose);
						}
					}
				}

				// now wait until we have a new image available according to our framerate
				if(num_frames % check_stop_file_every == 0) {
					run = existsRunFile();
				}

				double finish_time = (double)cv::getTickCount() * Config::time_ms_factor;
				int frame_duration = 1.0/1000.0 * (finish_time - start_time);
				clock_t print_sleep = 0;
				if(!image_processor.isLive() && Config::tracker_parameters.evaluation) {
					eval_timestamp += ms_per_frame;
				} else {
					if(wait_fps) {
						clock_t sleep_ms = ms_per_frame - frame_duration + ms_account_balance;
						if(sleep_ms > 0) {
							sleepMilliSeconds(sleep_ms);

							ms_account_balance = 0;	// reset account (account now positive again)
							print_sleep = sleep_ms * 1000;
						} else {
							if(-sleep_ms > ms_per_frame) {
								// we have to drop a frame
								ms_account_balance = 0;	// so we can keep up next frame in case we are faster
							} else {
								ms_account_balance += sleep_ms;	// so we can keep up next frame in case we are faster
							}
						}
					} else {
						ms_per_frame = frame_duration;
					}
				}

				// printout statistics
				if(!Config::tracker_parameters.demo) {
					line_buffer << num_frames << ",";
					line_buffer << after_grab - start_time << ",";
					line_buffer << after_preprocessing - after_grab << ",";
					line_buffer << after_detect - after_preprocessing << ",";
					line_buffer << after_identify - after_detect << ",";
					line_buffer << after_undistort - after_identify << ",";
					line_buffer << after_estimate - after_undistort << ",";
					line_buffer << finish_time - after_estimate << ",";
					line_buffer << finish_time - start_time << ",";
					line_buffer << print_sleep << endl;
					if(num_frames % 100 == 0) {
						benchmark_runtime << line_buffer.str();
						exposure_runtime << line_buffer2.str();
						line_buffer.clear();
						line_buffer.str("");
						line_buffer.str().reserve(20000);
						line_buffer2.clear();
						line_buffer2.str("");
						line_buffer2.str().reserve(10000);
					}
				}

				++num_frames;
			} else {
				fail_count++;
				if(fail_count > 3) {
					run = false;
				}
			}
		}

		if(Config::tracker_parameters.prediction) {
			cerr << "Prediction succeeded: " << predict_success << " frames" << endl;
			cerr << "Prediction failed: " << predict_failure << " frames" << endl;
			cerr << "Prediction success rate: " << (float)predict_success/(float)(predict_failure+predict_success-1) << endl;
			total_predict_success += predict_success;
			total_predict_failure += predict_failure;
		}
		total_no_marker_found += no_marker_found;
		cerr << "No marker found in " << no_marker_found << " frames" << endl;

		// fill data structure for mapping
		if(Config::tracker_parameters.mapping) {
			map<int, map<int, list<Pose2d>* const>* const> outer_map;
			for(list<MarkerRelation2D>::const_iterator it = map_update_list.begin(); it != map_update_list.end(); ++it) {
				// if we know about this marker
				if(Config::marker_parameters.base_T_m.count(it->dst_id)) {
					// maintain data structure
					if(outer_map.count(it->src_id) == 0) {
						map<int, list<Pose2d>* const>* const inner_map = new map<int, list<Pose2d>* const>();
						outer_map.insert(pair<int, map<int, list<Pose2d>* const> *const>(it->src_id, inner_map));
					}
					if(outer_map.at(it->src_id)->count(it->dst_id) == 0) {
						list<Pose2d>* const pose_list = new list<Pose2d>();
						outer_map.at(it->src_id)->insert(pair<int, list<Pose2d>* const>(it->dst_id, pose_list));
					}
					Pose2d m1_T_m2(it->dst_id, it->src_id, it->timestamp, 0, it->x, it->y, Config::marker_parameters.base_T_m.at(it->dst_id).translation3d[2], it->angle);
					outer_map.at(it->src_id)->at(it->dst_id)->push_back(m1_T_m2);
				}
			}
			compactMapData(outer_map);
			createMap();
			// cleanup
			for(map<int, map<int, list<Pose2d>* const>* const>::const_iterator outer_it = outer_map.begin(); outer_it != outer_map.end(); ++outer_it) {
				for(map<int, list<Pose2d>* const>::iterator inner_it = outer_it->second->begin(); inner_it != outer_it->second->end(); ++inner_it) {
					delete inner_it->second;
				}
				delete outer_it->second;
			}
		} else if(Config::tracker_parameters.evaluation) {
			// compute difference and put pose with that difference into the pose storage
			for(list<MarkerRelation2D>::const_iterator it = map_update_list.begin(); it != map_update_list.end(); ++it) {
				int src_id = it->src_id;
				int dst_id = it->dst_id;
				if(marker_relations.count(src_id)) {
					list<MarkerRelation2D>* const inner_list = marker_relations.at(src_id);
					for(list<MarkerRelation2D>::const_iterator inner_it = inner_list->begin(); inner_it != inner_list->end(); ++inner_it) {
						if(inner_it->dst_id == dst_id) {
							float x = (it->x - inner_it->x);
							float y = (it->y - inner_it->y);
							float yaw = it->angle - inner_it->angle;
							yaw += (yaw > g_PI) * -g_PI2;
							yaw += (yaw < -g_PI) * g_PI2;
							//float yaw = g_PI-FABS(g_PI-FABS(it->angle - inner_it->angle));

							Pose2d diff(dst_id, src_id, it->timestamp, it->error, x, y, 0, yaw);
							diff.count = it->corners_in_image;
							pose_storage6.add(diff);
							break;
						}
					}
				}
			}
		} else {
			// write collected map data down (can be used to create a better map over time)
			if(!map_update_list.empty()) {
				list<MarkerRelation2D>::const_iterator it = map_update_list.begin();
				cv::Mat map_update_matrix(map_update_list.size(), 6, CV_32F);
				for(int i = 0; i < map_update_matrix.rows; ++i) {
					float *map_row = map_update_matrix.ptr<float>(i);
					map_row[0] = it->timestamp;
					map_row[1] = it->src_id;
					map_row[2] = it->dst_id;
					map_row[3] = it->x;
					map_row[4] = it->y;
					map_row[5] = it->angle;
					++it;
				}
				map_update_list.clear();

				// current date and (local) time
				boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
				string date_iso_string = boost::posix_time::to_iso_string(now);
				string date_string = date_iso_string.substr(0, 15);
				string map_update_file("mapping_data_");
				map_update_file.append(date_string);
				map_update_file.append(".xml.gz");

				cv::FileStorage fs(map_update_file, cv::FileStorage::WRITE);
				fs << "date" << date_string;
				fs << "map_record" << map_update_matrix;
				fs.release();
			}
		}

		location_predictor->reset();
	}

	// write remaining stuff from buffer to disk
	benchmark_runtime << line_buffer.str();
	benchmark_runtime.close();
	exposure_runtime << line_buffer2.str();
	exposure_runtime.close();

	if(num_experiments > 1) {
		cerr << endl;
		cerr << "Total result:" << endl;
		if(Config::tracker_parameters.prediction) {
			cerr << "Prediction succeeded: " << total_predict_success << " frames" << endl;
			cerr << "Prediction failed: " << total_predict_failure-num_experiments << " frames" << endl;
			cerr << "Prediction success rate: " << (float)total_predict_success/(float)(total_predict_failure+total_predict_success-num_experiments) << endl;
		}
		cerr << "No marker found in " << total_no_marker_found << " frames" << endl;
	}
	cerr << endl;
	cerr << "number of poses erased: " << num_erased_poses << endl;
	cerr << "number of poses determined: " << num_poses << endl;

	// cleanup dynamic stuff
	if(pose_estimator != NULL)
		delete pose_estimator;
	if(location_predictor != NULL)
		delete location_predictor;
	for(map<int, list<MarkerRelation2D>* const>::iterator rel_it = marker_relations.begin(); rel_it != marker_relations.end(); ++rel_it) {
		rel_it->second->clear();
		delete rel_it->second;
	}

	// cleanup static stuff
	cleanupStaticStuff();
}
