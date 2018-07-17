/**
 * MeanAutoExposureAlgorithm.h
 *
 *	Determines new exposure time based on the mean pixel value
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#ifndef MATRIXAUTOEXPOSUREALGORITHM_H_
#define MATRIXAUTOEXPOSUREALGORITHM_H_

#include "autoexposurealgorithm.h"

namespace fml {

class MatrixAutoExposureAlgorithm : public AutoExposureAlgorithm {

protected:
	const int num_row_samples_ = 3;
	const int num_col_samples_ = 4;
	float target_average_;
	list<cv::Rect> rois_;

	virtual uint32_t computeSum(const cv::Mat &image) const;
	virtual float calcError(const cv::Mat &image, list<cv::Rect> &rois) const;

public:
	MatrixAutoExposureAlgorithm(const float min_val, const float max_val);
	virtual ~MatrixAutoExposureAlgorithm();

	virtual float getTargetValue() const;
	virtual void setTargetValue(const float value);
};

}
#endif /* MATRIXAUTOEXPOSUREALGORITHM_H_ */
