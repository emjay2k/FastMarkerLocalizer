/**
 * AutoSensorAlgorithm.h
 *
 *  Created on: 07.12.2012
 *      Author: jung
 */

#ifndef AUTOSENSORALGORITHM_H_
#define AUTOSENSORALGORITHM_H_

#include "generic/definitions.h"
#include "generic/types.h"

namespace fml {

class AutoSensorAlgorithm {

protected:
	float min_limit_;	// hardware min
	float max_limit_;	// hardware max
	float min_value_;	// arbitrary min value >= min_limit_
	float max_value_;	// arbitrary max value <= max_limit_
	float cur_value_;	// current value

	virtual bool update(const cv::Mat &image, list<cv::Rect> &rois) = 0;
public:
	AutoSensorAlgorithm(const float min_val, const float max_val);
	virtual ~AutoSensorAlgorithm();

	virtual void reset();
	virtual float getMin() const;
	virtual float getMax() const;
	virtual float getValue() const;
	virtual bool setMin(const float new_min);
	virtual bool setMax(const float new_max);
	virtual float setValue(const float value);

	virtual bool run(const cv::Mat &image, list<cv::Rect> &rois);
};

}
#endif /* AUTOSENSORALGORITHM_H_ */
