/**
 * ExpFilter.h
 *
 *	Implements a generic second order exponential filter
 *
 *  Created on: 05.04.2012
 *      Author: jung
 */

#ifndef EXPFILTER_H_
#define EXPFILTER_H_

namespace fml {

template <class T>
class ExpFilter {

	bool initialized_;
	T mean_;
	T mean_smooth_;
	float alpha_;

public:
	ExpFilter(const float alpha=0.5f) {
		initialized_ = false;
		alpha_ = alpha;
	}

	virtual ~ExpFilter() {
	}

	virtual void smooth(T& element) {
		if(!initialized_) {
			initialized_ = true;
			mean_ = element;
			mean_smooth_ = element;
		} else {
			mean_ = alpha_ * element + (1 - alpha_) * mean_;
			mean_smooth_ = alpha_ * mean_ + (1 - alpha_) * mean_smooth_;
			element = mean_smooth_;
		}
	}

	virtual void reset() {
		initialized_ = false;
	}

	virtual bool getInitialized() const {
		return(initialized_);
	}
};

}
#endif /* EXPFILTER_H_ */
