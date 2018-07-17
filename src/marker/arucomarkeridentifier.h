/*
 * arucomarkeridentifier.h
 *
 *  Created on: 03.02.2015
 *      Author: jung
 */

#ifndef SRC_MARKER_ARUCOMARKERIDENTIFIER_H_
#define SRC_MARKER_ARUCOMARKERIDENTIFIER_H_

#include <array>
#include <map>
//#include "aruco/arucofidmarkers.h"
//#include "aruco/highlyreliablemarkers.h"
#include "configure/markerparameters.h"
#include "marker/markeridentifier.h"
#include "generic/helper.h"

using namespace std;

namespace fml {

class ArucoMarkerIdentifier: public MarkerIdentifier {
	int (*identifier_function)(const cv::Mat &in, int &num_rotations);
	static uint8_t sum_map[1024];	// store the amount of white pixels per marker (key = id)
	static uint8_t* mat_map[1024]; // store the marker matrices and all possible rotations (key = id)

public:
	ArucoMarkerIdentifier();
	virtual ~ArucoMarkerIdentifier();

	void extractMarkerInfo(const cv::Mat &image, vector<cv::Point2f> &candidate, MarkerInfo &marker) const;
	void extractMarkerInfo(const cv::Mat &image, list<vector<cv::Point> > &candidates, list<MarkerInfo> &markers) const;

	static void rotate90_55(cv::Mat &in);
	static int hammDistMarker(const cv::Mat &bits); // copied from aruco for speed improvements
	static int identifyFidMarker(const cv::Mat &in, int &num_rotations); // copied from aruco for speed improvements
	static cv::Mat getMarkerMat(const int id); // copied from aruco for speed improvements
	static void init_static();
	static void clean_static();
};

} /* namespace fml */

#endif /* SRC_MARKER_ARUCOMARKERIDENTIFIER_H_ */
