/**
 * PoseToPayloadTranslator.h
 *
 * Export localization information via a datagram
 *
 *  Created on: 28.06.2012
 *      Author: jung
 */

#ifndef POSETOPAYLOADTRANSLATOR_H_
#define POSETOPAYLOADTRANSLATOR_H_

#include <array>
#include <vector>
#include "generic/helper.h"
#include "generic/expfilter.h"
#include "configure/config.h"

using namespace std;

namespace fml {

class PoseToPayloadTranslator {

	bool smooth_;
	ExpFilter<float> v_filter;
	ExpFilter<float> a_filter;

	array<char, 4> splitWordToBytes(const int value) const;
	array<char, 2> splitHalfwordToBytes(const short value) const;
	char signedChar(const unsigned char unsigned_char) const;
	short signedShort(const unsigned short unsigned_short) const;
	int signedInt(const unsigned int unsigned_int) const;

public:

	PoseToPayloadTranslator(const bool smooth=false);
	virtual ~PoseToPayloadTranslator();

	void translate(const Pose2d &pose, vector<char> &payload, float v_trans=0, float a_trans=0, const float v_rot=0, const float a_rot=0);
};

}
#endif /* POSETOPAYLOADTRANSLATOR_H_ */
