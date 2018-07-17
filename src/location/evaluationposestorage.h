/*
 * evaluationposestorage.h
 *
 *  Created on: 07.02.2015
 *      Author: jung
 */

#ifndef SRC_LOCATION_EVALUATIONPOSESTORAGE_H_
#define SRC_LOCATION_EVALUATIONPOSESTORAGE_H_

#include <list>
#include <fstream>
#include "location/simpleposestorage.h"

using namespace std;

namespace fml {

class EvaluationPoseStorage : public SimplePoseStorage {

	const double sigma_[6] = { 0.682689492, 0.954499736, 0.997300204, 0.99993666, 0.999999426697, 0.999999998027 };

	void write() const;

public:
	EvaluationPoseStorage(const string out, const bool enabled=true);
	virtual ~EvaluationPoseStorage();
};

} /* namespace fml */

#endif /* SRC_LOCATION_EVALUATIONPOSESTORAGE_H_ */
