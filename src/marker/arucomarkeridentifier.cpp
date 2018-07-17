/*
 * arucomarkeridentifier.cpp
 *
 *  Created on: 03.02.2015
 *      Author: jung
 */

#include "marker/arucomarkeridentifier.h"

namespace fml {

uint8_t ArucoMarkerIdentifier::sum_map[1024];
uint8_t* ArucoMarkerIdentifier::mat_map[1024];

ArucoMarkerIdentifier::ArucoMarkerIdentifier() {
	identifier_function = identifyFidMarker;
	//identifier_function = aruco::FiducidalMarkers::detect;
	//identifier_function = aruco::HighlyReliableMarkers::detect;
}

ArucoMarkerIdentifier::~ArucoMarkerIdentifier() {
}

void ArucoMarkerIdentifier::rotate90_55(cv::Mat &in) {
	uint8_t values[25];
	memcpy(values, in.data, 25*sizeof(uint8_t));
	uint8_t *data = in.data;
	data[0] = values[20];
	data[1] = values[15];
	data[2] = values[10];
	data[3] = values[5];
	data[4] = values[0];
	data[5] = values[21];
	data[6] = values[16];
	data[7] = values[11];
	data[8] = values[6];
	data[9] = values[1];
	data[10] = values[22];
	data[11] = values[17];
	data[12] = values[12];
	data[13] = values[7];
	data[14] = values[2];
	data[15] = values[23];
	data[16] = values[18];
	data[17] = values[13];
	data[18] = values[8];
	data[19] = values[3];
	data[20] = values[24];
	data[21] = values[19];
	data[22] = values[14];
	data[23] = values[9];
	data[24] = values[4];
}

int ArucoMarkerIdentifier::hammDistMarker(const cv::Mat &bits) {
	const int ids[4][5] = {{1, 0, 0, 0, 0}, {1, 0, 1, 1, 1}, {0, 1, 0, 0, 1}, {0, 1, 1, 1, 0}};
	int dist = 0;
	for(int y = 0; y < 5; ++y) {
		int minSum = 1e5;
		uchar *row = bits.data + bits.step*y;
		// hamming distance to each possible word
		for(int p = 0; p < 4; ++p) {
			// now, count
			int sum = (row[0] != ids[p][0])
            		+ (row[1] ^ ids[p][1])
					+ (row[2] ^ ids[p][2])
					+ (row[3] ^ ids[p][3])
					+ (row[4] ^ ids[p][4]);

			if(sum < minSum) {
				minSum = sum;
			}
		}
		// do the and
		dist += minSum;
	}

	return(dist);
}


int ArucoMarkerIdentifier::identifyFidMarker(const cv::Mat &in, int &num_rotations) {
	threshold(in, in, 125, 1, cv::THRESH_BINARY|cv::THRESH_OTSU);

	// now, analyze the interior in order to get the id
	// try first with the big ones

	// Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
	// the external border shoould be entirely black
	int swidth = in.rows/7;
	int swidth_cmp = (swidth * swidth) >> 1;

	// for first row, check the whole border
	/*for(int x = 0; x < 7; ++x) {
        if(countNonZero(in(cv::Rect(x*swidth, 0, swidth, swidth))) > swidth_cmp) {
            return(-1); // can not be a marker because the border element is not black!
        }
    }
    for(int y = 1; y < 6; y++) {
    	int y_swidth = y*swidth;
        for(int x = 0; x < 7; x += 6) {
            if(countNonZero(in(cv::Rect(x*swidth, y_swidth, swidth, swidth))) > swidth_cmp) {
                return(-1); // can not be a marker because the border element is not black!
            }
        }
    }
    // for last row, check the whole border
    int y_swidth = 6*swidth;
    for(int x = 0; x < 7; x ++) {
        if(countNonZero(in(cv::Rect(x*swidth, y_swidth, swidth, swidth))) > swidth_cmp) {
            return(-1); // can not be a marker because the border element is not black!
        }
    }*/

	// standard aruco approach
	cv::Mat bits(5, 5, CV_8UC1);
	// get information(for each inner square, determine if it is  black or white)
	/*for(int y = 0; y < 5; ) {
		int next_y = y + 1;
		int Ystart = next_y * swidth;
		uchar *row = bits.data + bits.step*y;
		for(int x = 0; x < 5; ) {
			int next_x = x + 1;
			row[x] = countNonZero(in(cv::Rect(next_x*swidth, Ystart, swidth, swidth))) > swidth_cmp;
			x = next_x;
		}
		y = next_y;
	}*/

	// super-fast alternative method (faster because we have small matrix sizes, this is faster (up to 56x56)
	int log2_sql = 0;	// log2 of square length. since this is a power of 2, we can shift by it instead of dividing by square_length
	int divisor = swidth;
	while(divisor > 1) {
		divisor >>= 1;
		++log2_sql;
	}
	int inner_size = in.rows - 2*swidth;
	cv::Mat decode_area(in, cv::Rect(swidth, swidth, inner_size, inner_size));
	cv::Mat current_mat(5, 5, CV_16U, cv::Scalar(0));
	for(int i = 0; i < inner_size; ++i) {
		const uint8_t* decode_row = decode_area.ptr<uint8_t>(i);
		uint16_t* current_row = current_mat.ptr<uint16_t>(i >> log2_sql);
		for(int j = 0; j < inner_size; ++j) {
			current_row[j >> log2_sql] += decode_row[j];
		}
	}

	// determine mean value of each cell
	uint16_t* current_val = (uint16_t*)current_mat.data;
	uint8_t* bits_val = bits.data;
	for(int i = 0; i < 25; ++i) {
		bits_val[i] = current_val[i] > swidth_cmp;
	}

	// check all possible rotations
	num_rotations = 0;
	int best_dist = hammDistMarker(bits);
	if(best_dist > 0) {
		for(int i = 1; i < 4; ++i) {
			// rotate
			rotate90_55(bits);
			// get the hamming distance to the nearest possible word
			best_dist = hammDistMarker(bits);
			// stop if we have found a solution already
			if(best_dist == 0) {
				num_rotations = i;
				break;
			}
		}
	}

	if(best_dist == 0) {
		int MatID = 0;
		for(int y = 0; y < 5; ++y) {
			uchar *row = bits.data + bits.step*y;
			MatID <<= 1;
			MatID |= row[1];
			MatID <<= 1;
			MatID |= row[3];
		}
		return(MatID);
	} else { // Get id of the marker
		return(-1);
	}
}

cv::Mat ArucoMarkerIdentifier::getMarkerMat(const int id) {
	if(0 <= id && id < 1024) {
		cv::Mat marker(5, 5, CV_8UC1);
		// for each line, create
		const int ids[4] = {0x10, 0x17, 0x09, 0x0e};
		for(int y = 0; y < 5; ++y) {
			int val = ids[(id >> 2 * (4 - y)) & 0x0003];
			uchar *row = marker.data + marker.step*y;
			for(int x = 0; x < 5; ++x) {
				row[x] = (val >> (4 - x)) & 0x0001;
			}
		}
		return(marker);
	}
	return(cv::Mat());
}

void ArucoMarkerIdentifier::extractMarkerInfo(const cv::Mat &image, list<vector<cv::Point> > &candidates, list<MarkerInfo> &markers) const {
	map<int, vector<cv::Point2f> > detected_markers;

	for(list<vector<cv::Point> >::const_iterator it = candidates.begin(); it != candidates.end(); ++it) {
		//orderClockwise(*it);	// no longer necessary

		vector<cv::Point2f> candidate;
		candidate.reserve(4);
		candidate.push_back(cv::Point2f((*it)[0].x, (*it)[0].y));
		candidate.push_back(cv::Point2f((*it)[1].x, (*it)[1].y));
		candidate.push_back(cv::Point2f((*it)[2].x, (*it)[2].y));
		candidate.push_back(cv::Point2f((*it)[3].x, (*it)[3].y));

		// find projective homography and unwarp
		int num_rotations;
		cv::Mat canonicalMarker;
		warp_helper(image, canonicalMarker, candidate);

		int quarter_size = canonicalMarker.cols >> 1;
		cv::Mat q1(canonicalMarker, cv::Rect(0, 0, quarter_size, quarter_size));
		fastNormalize2(q1);
		cv::Mat q2(canonicalMarker, cv::Rect(0, quarter_size, quarter_size, quarter_size));
		fastNormalize2(q2);
		cv::Mat q3(canonicalMarker, cv::Rect(quarter_size, 0, quarter_size, quarter_size));
		fastNormalize2(q3);
		cv::Mat q4(canonicalMarker, cv::Rect(quarter_size, quarter_size, quarter_size, quarter_size));
		fastNormalize2(q4);

		int id = (*identifier_function) (canonicalMarker, num_rotations);

		if(id > -1) {
			// deletes the one with smaller perimeter
			// TODO: shouldnt we delete the bigger one and keep the smaller?
			bool add_marker = true;
			if(detected_markers.count(id)) {	// marker already found in the same image
				if(computePerimeter(detected_markers[id]) < computePerimeter(candidate)) {
					detected_markers.erase(id);
				} else {
					add_marker = false;
				}
			}
			if(add_marker) {	// add new marker
				// sort the points so they are always in the same order no matter the camera orientation
				rotate(candidate.begin(), candidate.begin()+(4-num_rotations), candidate.end());
				detected_markers.insert(pair<int, vector<cv::Point2f> >(id, candidate));
			}
		}
	}

	// add located markers to the list
	for(map<int, vector<cv::Point2f> >::const_iterator it = detected_markers.begin(); it != detected_markers.end(); ++it) {
		Pose2d p(it->first, 0, 0, 0, 0, 0, 0, 0);
		// create MarkerInfo object
		MarkerInfo mi = MarkerInfo(0, p);
		// copy vertex information (important)
		copy(it->second.begin(), it->second.end(), mi.vertices);
		markers.push_back(mi);
	}
}

void ArucoMarkerIdentifier::extractMarkerInfo(const cv::Mat &image, vector<cv::Point2f> &candidate, MarkerInfo &marker) const {
	//orderClockwise(candidate); // no longer necessary

	// find projective homography and unwarp
	int num_rotations = 0;
	cv::Mat canonicalMarker;
	warp_helper(image, canonicalMarker, candidate);

	if(marker.id > -1) {
		// get the area with the inner id
		int square_length = marker_warp_size_/7;
		int inner_size = marker_warp_size_ - 2 * square_length;
		int quarter_size = inner_size >> 1;

		int log2_sql = 0;	// log2 of square length. since this is a power of 2, we can shift by it instead of dividing by square_length
		int divisor = square_length;
		while(divisor > 1) {
			divisor >>= 1;
			++log2_sql;
		}
		int log2_sqa = 2 * log2_sql; // log2 of square area. since this is also a power of 2, we can shift by it instead of dividing by square_area

		int sl_qs = square_length + quarter_size;
		cv::Mat q1(canonicalMarker, cv::Rect(square_length, square_length, quarter_size, quarter_size));
		fastNormalize2(q1);
		cv::Mat q2(canonicalMarker, cv::Rect(sl_qs, square_length, quarter_size, quarter_size));
		fastNormalize2(q2);
		cv::Mat q3(canonicalMarker, cv::Rect(square_length, sl_qs, quarter_size, quarter_size));
		fastNormalize2(q3);
		cv::Mat q4(canonicalMarker, cv::Rect(sl_qs, sl_qs, quarter_size, quarter_size));
		fastNormalize2(q4);

		cv::Mat decode_area(canonicalMarker, cv::Rect(square_length, square_length, inner_size, inner_size));
		cv::Mat current_mat(5,5,CV_16U,cv::Scalar(0));
		for(int i = 0; i < inner_size; ++i) {
			const uint8_t* decode_row = decode_area.ptr<uint8_t>(i);
			uint16_t* current_row = current_mat.ptr<uint16_t>(i >> log2_sql);
			for(int j = 0; j < inner_size; ++j) {
				current_row[j >> log2_sql] += decode_row[j];
			}
		}

		// determine mean value of each cell
		cv::Mat mean_mat(5,5,CV_8UC1);
		uint16_t* current_val = (uint16_t*)current_mat.data;
		uint8_t* mean_val = mean_mat.data;
		for(int i = 0; i < 25; ++i) {
			mean_val[i] = (uint8_t)(current_val[i] >> log2_sqa);
		}

		// determine suitable threshold for this specific marker
		/*uint8_t num_white_pixels = sum_map[marker.id];

		uint8_t values[25];
		memcpy(values, mean_val, 25*sizeof(uint8_t));
		int pos = 24 - num_white_pixels;
		nth_element(values, values+pos, values+25);
		uint8_t threshold = values[pos];
		for(int i = 0; i < 25; ++i) {
			mean_val[i] = mean_val[i] > threshold;
		}*/

		// otsu thresholding is comparable to the invented method
		threshold(mean_mat, mean_mat, 125, 1, cv::THRESH_BINARY|cv::THRESH_OTSU);

		uint8_t* marker_mat = mat_map[marker.id];
		int deviation = 0;
		// get hamming distance to the expected id (taken from aruco)
		for(int j = 0; j < 25; ++j) {
			deviation += marker_mat[j] ^ mean_val[j];
		}

		marker.detection_error = deviation;
	} else {
		int quarter_size = canonicalMarker.cols >> 1;
		cv::Mat q1(canonicalMarker, cv::Rect(0, 0, quarter_size, quarter_size));
		fastNormalize2(q1);
		cv::Mat q2(canonicalMarker, cv::Rect(0, quarter_size, quarter_size, quarter_size));
		fastNormalize2(q2);
		cv::Mat q3(canonicalMarker, cv::Rect(quarter_size, 0, quarter_size, quarter_size));
		fastNormalize2(q3);
		cv::Mat q4(canonicalMarker, cv::Rect(quarter_size, quarter_size, quarter_size, quarter_size));
		fastNormalize2(q4);
		// repairing is no longer necessary, as we modified the identifier function ourselves
		// repair marker border (1/2 the real border), in case it accidentally got screwed up
		/*int border_size = marker_warp_size_.width/(7*2)+1;
		int max_pixel = canonicalMarker.rows-1;
		int max_pixel_border = max_pixel - border_size;
		cv::rectangle(canonicalMarker, cv::Point(0,0), cv::Point(max_pixel, border_size), cv::Scalar(0), CV_FILLED);
		cv::rectangle(canonicalMarker, cv::Point(0,max_pixel_border), cv::Point(max_pixel, max_pixel), cv::Scalar(0), CV_FILLED);
		cv::rectangle(canonicalMarker, cv::Point(0,border_size), cv::Point(border_size, max_pixel_border), cv::Scalar(0), CV_FILLED);
		cv::rectangle(canonicalMarker, cv::Point(max_pixel_border,border_size), cv::Point(max_pixel, max_pixel_border), cv::Scalar(0), CV_FILLED);*/

		marker.id = (*identifier_function) (canonicalMarker, num_rotations);
	}

	if(marker.id > -1) {
		// sort the points so they are always in the same order no matter the camera orientation
		rotate(candidate.begin(), candidate.begin()+(4-num_rotations), candidate.end());
		// copy vertex information (important)
		copy(candidate.begin(), candidate.end(), marker.vertices);
	}
}

void ArucoMarkerIdentifier::init_static() {
	// initialize sum_map
	memset(sum_map, 0, 1024 * sizeof(sum_map[0]));
	// fill sum and mat maps
	for(map<int, Pose2d>::const_iterator it = Config::marker_parameters.base_T_m.begin(); it != Config::marker_parameters.base_T_m.end(); ++it) {
		cv::Mat tmp = getMarkerMat(it->first);
		sum_map[it->first] = (uint8_t)cv::sum(tmp).val[0];
		uint8_t* marker_mat = new uint8_t[25];
		memcpy(marker_mat, tmp.data, 25*sizeof(uint8_t));
		mat_map[it->first] = marker_mat;
	}
}

void ArucoMarkerIdentifier::clean_static() {
	// erase buffered matrices
	for(size_t i = 0; i < 1024; ++i) {
		if(sum_map[i] > 0) {
			delete[] mat_map[i];
		}
	}
}

} /* namespace fml */
