/**
 * AutoExposureAlgorithm.h
 *
 *	Computes new exposure time for AE
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#ifndef AUTOEXPOSUREALGORITHM_H_
#define AUTOEXPOSUREALGORITHM_H_

#include "autosensoralgorithm.h"

namespace fml {

class AutoExposureAlgorithm : public AutoSensorAlgorithm {

protected:
	float exposure_threshold_;

	virtual float calcError(const cv::Mat &image, list<cv::Rect> &rois) const = 0;
	virtual bool update(const cv::Mat &image, list<cv::Rect> &rois);

public:
	AutoExposureAlgorithm(const float min_val, const float max_val);
	virtual ~AutoExposureAlgorithm();

	float getThreshold() const;
	void setThreshold(const float value);
};

}
#endif /* AUTOEXPOSUREALGORITHM_H_ */
