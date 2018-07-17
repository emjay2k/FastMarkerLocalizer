/*
 * markerextractor.h
 *
 *  Created on: 01.08.2015
 *      Author: Jung
 */

#ifndef SRC_FULLMARKEREXTRACTOR_H_
#define SRC_FULLMARKEREXTRACTOR_H_

#include "marker/markerextractor.h"

namespace fml {

class FullMarkerExtractor : public MarkerExtractor {

	const float detection_opp_length_threshold = (8.0*8.0)/(7.0*7.0);

	ArucoMarkerDetector detector_;
	ArucoMarkerIdentifier identifier_;

	bool keepCandidate(const vector<cv::Point> &candidate) const;

public:
	FullMarkerExtractor();
	virtual ~FullMarkerExtractor();

	list<MarkerInfo> getMarkers(cv::Mat &image, double &after_preprocessing, double &after_detect, double &after_identify);
};

} /* namespace fml */

#endif /* SRC_FULLMARKEREXTRACTOR_H_ */
