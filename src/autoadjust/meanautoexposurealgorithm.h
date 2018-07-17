/**
 * MeanAutoExposureAlgorithm.h
 *
 *	Determines new exposure time based on the mean pixel value
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#ifndef MEANAUTOEXPOSUREALGORITHM_H_
#define MEANAUTOEXPOSUREALGORITHM_H_

#include "autoexposurealgorithm.h"

namespace fml {

class MeanAutoExposureAlgorithm : public AutoExposureAlgorithm {

protected:
	float target_average_;

	virtual uint32_t computeSum(const cv::Mat &image) const;
	virtual float calcError(const cv::Mat &image, list<cv::Rect> &rois) const;

public:
	MeanAutoExposureAlgorithm(const float min_val, const float max_val);
	virtual ~MeanAutoExposureAlgorithm();

	virtual float getTargetValue() const;
	virtual void setTargetValue(const float value);
};

}
#endif /* MEANAUTOEXPOSUREALGORITHM_H_ */
