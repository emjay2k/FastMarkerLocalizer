/*
 * imgproc.h
 *
 *  Created on: 27.06.2015
 *      Author: jung
 */

#ifndef SRC_IMGPROC_H_
#define SRC_IMGPROC_H_

#include <opencv2/opencv.hpp>
#include "generic/definitions.h"

using namespace std;

namespace fml {

/**
 * Based on the original opencv code, just faster to compute
 *
 * @param src
 * @param dst
 */
inline void binning2x2(const cv::Mat &src, cv::Mat &dst) {
	cv::Size size(src.size().width >> 1, src.size().height >> 1);
	dst.create(size, src.type());

	size_t src_step_offset = 2*src.step;
	for(int i = 0; i < size.height; ++i) {
		const uint8_t *s_row1 = src.data + src_step_offset*i;
		const uint8_t *s_row2 = s_row1 + src.step;
		uint8_t *d_row = dst.data + dst.step*i;
		int j = 0;
		for( ; j < size.width-3; j += 4) {
			int v0 = s_row1[0], v1 = s_row1[1], v2 = s_row2[0], v3 = s_row2[1];
			uint8_t x0 = (v0 + v1 + v2 + v3) >> 2;
			v0 = s_row1[2], v1 = s_row1[3], v2 = s_row2[2], v3 = s_row2[3];
			uint8_t x1 = (v0 + v1 + v2 + v3) >> 2;
			d_row[0] = x0, d_row[1] = x1;

			v0 = s_row1[4], v1 = s_row1[5], v2 = s_row2[4], v3 = s_row2[5];
			x0 = (v0 + v1 + v2 + v3) >> 2;
			v0 = s_row1[6], v1 = s_row1[7], v2 = s_row2[6], v3 = s_row2[7];
			x1 = (v0 + v1 + v2 + v3) >> 2;
			d_row[2] = x0, d_row[3] = x1;

			s_row1 += 8;
			s_row2 += 8;
			d_row += 4;
		}
		for( ; j < size.width; ++j) {
			int v0 = s_row1[0], v1 = s_row1[1];
			int v2 = s_row2[0], v3 = s_row2[1];
			d_row[0] = (uint8_t)((v0 + v1 + v2 + v3) >> 2);

			s_row1 += 2;
			s_row2 += 2;
			++d_row;
		}
	}
}

// hopefully this method is faster for our relevant case
inline void fastNormalize2(cv::Mat &image) {
	cv::Size size = image.size();
	if(image.isContinuous()) {
		size.width *= size.height;
		size.height = 1;
	}

	// determine min and max efficiently by marking all the values that occur:
	// this is similar to a histogram, but we only mark the occurrence (table[value] > 0)
	uint8_t table[256] = { 0 };
	for(int i = 0; i < size.height; ++i) {
		const uint8_t* p = image.data + image.step*i;
		int j = 0;
		for( ; j < size.width-3; j += 4) {
			uint8_t v0 = p[0], v1 = p[1];
			table[v0] = 1; table[v1] = 1;
			v0 = p[2]; v1 = p[3];
			table[v0] = 1; table[v1] = 1;

			p += 4;
		}
		for( ; j < size.width; ++j) {
			int v = p[0];
			table[v] = 1;

			++p;
		}
	}

	// determine min and max from the values in table
	uint8_t smin = 0, smax = 255;
	while(likely(!table[smin])) {
		++smin;
	}

	while(likely(!table[smax])) {
		--smax;
	}

	int spectrum = smax - smin;
	if(likely(spectrum > 0 && spectrum < 255)) {
		uint32_t alpha = (int)((65536.0f * 255.f)/(float)spectrum);

		for(int i = 0; i < size.height; ++i) {
			uint8_t* p = image.data + image.step*i;
			int j = 0;
			for( ; j < size.width-3; j += 4) {
				uint32_t v0 = p[0], v1 = p[1];
				uint8_t x0 = (uint8_t)((alpha * (v0 - smin)) >> 16);
				uint8_t x1 = (uint8_t)((alpha * (v1 - smin)) >> 16);
				p[0] = x0, p[1] = x1;
				v0 = p[2], v1 = p[3];
				x0 = (uint8_t)((alpha * (v0 - smin)) >> 16);
				x1 = (uint8_t)((alpha * (v1 - smin)) >> 16);
				p[2] = x0, p[3] = x1;

				p += 4;
			}
			for( ; j < size.width; ++j) {
				uint32_t v = p[0];
				p[0] = (uint8_t)((alpha * (v - smin)) >> 16);

				++p;
			}
		}
	} // else do nothing, as we have either only one color or all colors already
}

}

#endif /* SRC_IMGPROC_H_ */
