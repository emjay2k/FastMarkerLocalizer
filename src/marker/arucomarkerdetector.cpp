/*
 * arucomarkerdetector.cpp
 *
 *  Created on: 03.02.2015
 *      Author: jung
 */

#include "marker/arucomarkerdetector.h"

namespace fml {

int ArucoMarkerDetector::size_template_[65536];
uint16_t ArucoMarkerDetector::label_template_[65536];

ArucoMarkerDetector::ArucoMarkerDetector() {
	tl_ = new cv::Point[65536];
	br_ = new cv::Point[65536];
	// initialize
	memcpy(&size_, &size_template_, 65536*sizeof(int));
	memcpy(&L_, &label_template_, 65536*sizeof(uint16_t));
}

ArucoMarkerDetector::~ArucoMarkerDetector() {
	delete[] tl_;
	delete[] br_;
}

/**
 * This code has a problem: the hull is correctly determined, but unfortunately the points are not in the correct order.
 * Hence i run OpenCV's convexHull on the hull to order it;
 * 
 * @param P
 * @param H
 */
void ArucoMarkerDetector::convexHullMC(const CvSeq* const P, vector<CvPoint*> &H) const {
	int n = P->total, k = 0;
	H.resize(2*n);

	// Build lower hull
	for(int i = 0; i < n; ++i) {
		while(k >= 2 && crossProduct(H[k-2], H[k-1], (CvPoint*)cvGetSeqElem(P, i)) >= 0) {
			--k;
		}
		H[k++] = (CvPoint*)cvGetSeqElem(P, i);
	}

	// Build upper hull
	for(int i = n-2, t = k+1; i >= 0; --i) {
		while(k >= t && crossProduct(H[k-2], H[k-1], (CvPoint*)cvGetSeqElem(P, i)) >= 0) {
			--k;
		}
		H[k++] = (CvPoint*)cvGetSeqElem(P, i);
	}

	H.resize(k-1);
}

void ArucoMarkerDetector::search_part(const cv::Mat &image, list<vector<cv::Point> > &candidates, const int factor_pyr) {
	// we start with a small image to begin with and need similar values directly
	// parameters are the default ones taken from aruco
	int threshold_block_size = 7;
	double threshold_C = 7;

	size_t min_perimeter = min_perimeter_, max_perimeter = max_perimeter_;
	size_t min_length_p2 = min_length_p2_;

	if(factor_pyr > 1) {
		// new constraint values
		min_perimeter /= factor_pyr;
		max_perimeter /= factor_pyr;
		min_length_p2 /= factor_pyr * factor_pyr;	// because we are dealing with the squared value
		threshold_block_size /= factor_pyr;
		threshold_C /= factor_pyr;
	}

	// the number of contour points does not necessarily represent the contours length.
	// if all connections are diagonal, then the distance between two subsequent contour
	// points is sqrt(2) instead of 1, so we have to divide by 1/sqrt(2) to be safe.
	cv::Mat binary_image;
	//double before = (double)cv::getTickCount() * 1000000.0/cv::getTickFrequency();
	//adaptiveThreshold(image, binary_image, 1, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, ADAPTIVE_THRESHOLD_BLOCKSIZE, ADAPTIVE_THRESHOLD_DELTA);
	//fastAdaptiveThreshold(image, binary_image);
	integratedAdaptiveThreshold(image, binary_image);
	//double after = (double)cv::getTickCount() * 1000000.0/cv::getTickFrequency();
	int min_length_estimate = min_perimeter >> 2;
	int max_length_estimate = (float)sqrt_2 * (max_perimeter >> 2);
	filterBinary(binary_image, min_length_estimate, max_length_estimate, max_perimeter, candidates);
	//integratedfindCandidate(image, min_length_estimate, max_length_estimate, max_perimeter, candidates);
	//double after2 = (double)cv::getTickCount() * 1000000.0/cv::getTickFrequency();
	//cout << after-before << "," << after2-after << endl;

	// this is now the part of the program consuming the most significant amount of ressources
	// other than migrating it to the C-Api (which actually helped), I couldnt improve performance without hurting robustness too much
	// tried to prefilter the images in order to remove noise, but nothing really helped here. Probably gonna be stuck there so I dont bother anymore
	/*size_t min_perimeter_estimate = 1/sqrt_2 * min_perimeter;
	CvSeq *first_contour = NULL;
	CvMemStorage *storage = cvCreateMemStorage();
	IplImage *binary_ipl = new IplImage(binary_image);	// do not release this!
	cvFindContours(binary_ipl, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	delete binary_ipl;

	for(CvSeq *cv_contour = first_contour; cv_contour != NULL; cv_contour = cv_contour->h_next) {
		// estimate and enforce contour length constraints (with some laxity)
		// up to 2 comparisons with lookup values (drops too small or too large polygons)
		size_t num_points = cv_contour->total;
		if(unlikely(num_points >= min_perimeter_estimate && num_points <= max_perimeter)) {
			//size_t contour_length = fastContourLength(cv_contour, true);
			// up to 2 comparisons with lookup values (drops too small or too large polygons)
			size_t num_points_2 = num_points >> 1;
			size_t num_points_4 = num_points >> 2;
			size_t num_points_8 = num_points >> 3;
			// drop polygons early that have at least one strongly acute angle
			// the idea is to take 4 points from the contour (at beginning, 1/4, 1/2 and 3/4 of the perimeter and another set with 1/8 offset).
			// the distance between them should not be smaller than the smallest allowed angle (conservative)
			// 0, 1/4, 1/2, 3/4 positions along the polygonal line (each step +1/4)
			// 1/8, 3/8, 5/8, 7/8 positions along the polygonal line (each step +1/4). Same as above with 1/8*num_points offset.
			// up to 4 comparisons, each involving 3 additions and two multiplications
			if(unlikely(squared_distance((CvPoint*)cvGetSeqElem(cv_contour, num_points_4), (CvPoint*)cvGetSeqElem(cv_contour, num_points-num_points_4)) >= min_length_p2
					&& squared_distance((CvPoint*)cvGetSeqElem(cv_contour, 0), (CvPoint*)cvGetSeqElem(cv_contour, num_points_2)) >= min_length_p2
					&& squared_distance((CvPoint*)cvGetSeqElem(cv_contour, num_points_8), (CvPoint*)cvGetSeqElem(cv_contour, num_points_2+num_points_8)) > min_length_p2
					&& squared_distance((CvPoint*)cvGetSeqElem(cv_contour, num_points_4+num_points_8), (CvPoint*)cvGetSeqElem(cv_contour, num_points-num_points_8)) > min_length_p2)) {
				// now compute actual length and enforce the true length constraints
				size_t contour_length = fastContourLength(cv_contour, true);
				if(likely(contour_length <= max_perimeter && contour_length >= min_perimeter)) {
					CvSeq *hull = cvConvexHull2(cv_contour, storage, CV_CLOCKWISE, 1);
					CvSeq *approx_curve = cvApproxPoly(hull, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cv_contour->total*0.05);
					//approxQuad(cv_contour, square);	// dont use that anymore
					if(approx_curve->total == 4) {
						// approxPolyDP has reversed the clockwise order, hence we have to reverse it back
						vector<cv::Point> square;
						square.reserve(4);
						square.push_back(*(CvPoint*)cvGetSeqElem(approx_curve, 0));
						square.push_back(*(CvPoint*)cvGetSeqElem(approx_curve, 3));						
						square.push_back(*(CvPoint*)cvGetSeqElem(approx_curve, 2));
						square.push_back(*(CvPoint*)cvGetSeqElem(approx_curve, 1));
						candidates.push_back(square);
					}
				}
			}
		}
	}

	cvReleaseMemStorage(&storage);*/
}

/**
 * Rewrite this to only search the rois until a marker is found (ideally only once)
 *
 * @param image
 * @param rois
 * @param candidates
 */
void ArucoMarkerDetector::findSquares(const cv::Mat &image, list<vector<cv::Point> > &candidates) {
	if(pyr_down_) {
		// new threshold values
		const int factor_pyr = 2;
		cv::Mat downsampled;
		// resize image to 1/4 for finding corner values
		//pyrDown(image, downsampled);
		//resize(image, downsampled, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
		binning2x2(image, downsampled);
		// aruco uses a general means of pyrdown, however for any reasonable image system (2mp and below), this will not be necessary more than once, so why bother
		search_part(downsampled, candidates, factor_pyr);

		// update values to map them to the original image
		for(list<vector<cv::Point> >::iterator it = candidates.begin(); it != candidates.end(); ++it) {
			(*it)[0] *= factor_pyr;
			(*it)[1] *= factor_pyr;
			(*it)[2] *= factor_pyr;
			(*it)[3] *= factor_pyr;
		}
	} else {
		// search entire image at full resolution
		search_part(image, candidates, 1);
	}
}

void ArucoMarkerDetector::integratedAdaptiveThreshold(const cv::Mat &src, cv::Mat &dst) const {
	cv::Mat tmp(src.size(), CV_16U);
	dst.create(src.size(), src.type());

	const int offset = ADAPTIVE_THRESHOLD_BLOCKSIZE >> 1;
	const int offset_inc = offset + 1;
	int max_row = src.rows;
	int max_col = src.cols;
	int num_middle_cols = max_col - 2*offset - 3;
	int num_middle_rows = max_row - 2*offset - 3;
	const int thresh_window_size = ADAPTIVE_THRESHOLD_BLOCKSIZE * ADAPTIVE_THRESHOLD_BLOCKSIZE;
	const int thresh_offset = thresh_window_size * ADAPTIVE_THRESHOLD_DELTA;

	// first the columns
	const uint8_t *sdata_add = src.data;
	const uint8_t *sdata_sub = src.data;
	uint16_t *tdata = (uint16_t*)tmp.data;
	for(int i = 0; i < max_row; ++i) {
		// first element
		int left_border = *sdata_add;	// sdata[0]
		++sdata_add;	// sdata[1]
		sdata_sub = sdata_add; // sdata[1]
		int sum = offset_inc * left_border;
		for(int j = 0; j < offset; ++j) {
			sum += *sdata_add; 	// sdata[j]
			++sdata_add;	// sdata[j+1]
		}
		*tdata = (uint16_t)sum;
		++tdata;	// tdata[1]
		// next elements which contain parts of the replicated border
		for(int j = 0; j < offset_inc; ++j) {
			sum = sum + *sdata_add - left_border;
			*tdata = (uint16_t)sum;
			++sdata_add;
			++tdata;
		}

		// all the middle elements, that are not in touch with the border
		for(int j = 0; j < num_middle_cols; ++j) {
			sum = sum + *sdata_add - *sdata_sub;
			*tdata = (uint16_t)sum;
			++sdata_add;
			++sdata_sub;
			++tdata;
		}

		// later elements
		int right_border = *sdata_add;
		for(int j = 0; j < offset_inc; ++j) {
			sum = sum + right_border - *sdata_sub;
			*tdata = (uint16_t)sum;
			++sdata_sub;
			++tdata;
		}

		++sdata_add;
	}

	// now the rows
	// begin of our buffer
	int *line_buffer = new int[max_col];
	int* const buffer_begin = line_buffer;
	int *buffer = buffer_begin;

	const uint8_t *sdata = src.data;
	uint8_t *ddata = dst.data;
	const uint16_t *tmp_add = (uint16_t*)tmp.data;
	const uint16_t *tmp_sub = (uint16_t*)tmp.data;
	// first row
	for(int j = 0; j < max_col; ++j) {
		*buffer = offset_inc * (*tmp_add);
		++buffer;
		++tmp_add;
	}
	for(int i = 0; i < offset; ++i) {
		buffer = buffer_begin;
		for(int j = 0; j < max_col; ++j) {
			*buffer += *tmp_add;
			++buffer;
			++tmp_add;
		}
	}
	buffer = buffer_begin;
	for(int j = 0; j < max_col; ++j) {
		int value = thresh_window_size * (*sdata) + thresh_offset;
		*ddata = value <= *buffer;	// use MAC where possibleC
		++buffer;
		++sdata;
		++ddata;
	}

	// next rows which contain parts of the replicated border
	const uint16_t *tmp_sub_firstrow_begin = tmp_sub;
	for(int i = 0; i < offset_inc; ++i) {	// rows
		buffer = buffer_begin;
		tmp_sub = tmp_sub_firstrow_begin;
		for(int j = 0; j < max_col; ++j) {	// cols
			*buffer = *buffer + *tmp_add - *tmp_sub;
			int value = thresh_window_size * (*sdata) + thresh_offset;
			*ddata = value <= *buffer;
			++buffer;
			++tmp_add;
			++tmp_sub;
			++sdata;
			++ddata;
		}
	}

	// all the middle elements, that are not in touch with the border
	for(int i = 0; i < num_middle_rows; ++i) {
		buffer = buffer_begin;
		for(int j = 0; j < max_col; ++j) {
			*buffer = *buffer + *tmp_add - *tmp_sub;
			int value = thresh_window_size * (*sdata) + thresh_offset;
			*ddata = value <= *buffer;
			++buffer;
			++tmp_add;
			++tmp_sub;
			++sdata;
			++ddata;
		}
	}

	// add last rows
	const uint16_t *tmp_add_lastrow_begin = tmp_add;
	for(int i = 0; i < offset_inc; ++i) {
		buffer = buffer_begin;
		for(int j = 0; j < max_col; ++j) {
			*buffer = *buffer + *tmp_add - *tmp_sub;
			int value = thresh_window_size * (*sdata) + thresh_offset;
			*ddata = value <= *buffer;
			++buffer;
			++tmp_add;
			++tmp_sub;
			++sdata;
			++ddata;
		}

		// update rows
		tmp_add = tmp_add_lastrow_begin;
	}

	delete[] line_buffer;
}

void ArucoMarkerDetector::filterBinary(const cv::Mat &image, const int min_length_estimate, const int max_length_estimate, const int max_perimeter, list<vector<cv::Point> > &candidates) {
	// erase values of first and last row/col
	// first and last row
	int max_row = image.rows;
	int max_col = image.cols;

	// label the components
	// implementation from the paper
	//cv::Mat pixel_labels(max_row, max_col, CV_16U, cv::Scalar(0));	// beware we use 16-bit (unsigned short) per pixel here, as there can be more than 256 connected regions at first
	// setting all to zero in one swoop is faster than cv::Mat constructor above
	cv::Mat pixel_labels(max_row, max_col, CV_16U);
	memset(pixel_labels.data, 0, max_row*max_col*sizeof(uint16_t));
	// 8-way connected component analysis
	// pass one
	uint16_t label = 0;	// label no.

	// current search window
	const uint8_t *p0 = image.data;	// top_left pixel
	const uint8_t *p1 = image.data;	// top pixel
	const uint8_t *p2 = image.data;	// left pixel
	const uint8_t *p3 = image.data;	// current pixel
	uint16_t *l0 = (uint16_t*)pixel_labels.data;	// top_left label
	uint16_t *l1 = (uint16_t*)pixel_labels.data;	// top label
	uint16_t *l2 = (uint16_t*)pixel_labels.data;	// left label
	uint16_t *l3 = (uint16_t*)pixel_labels.data;	// current label

	// first row: first pixel
	if(unlikely(*p3 > 0)) {
		++label;
		*l3 = label;
		// update the datastructure
		cv::Point region_start(0,0);
		tl_[label] = region_start;
		br_[label] = region_start;
	}
	++p3;
	++l3;

	// first row: rest
	for(int i = 1; i < max_col; ++i) {
		if(unlikely(*p3 > 0)) {
			if(*p2 == 0) {
				++label;
				*l3 = label;

				cv::Point region_start(i,0);
				tl_[label] = region_start;
				br_[label] = region_start;
			} else {
				uint16_t left_label = *l2;
				// left label couldn't have been merged before, because it would have been labeled after merging => image_row[i_prev] is not necessary
				*l3 = left_label;
				++size_[left_label];
				// we have a bigger x value for sure
				br_[left_label].x = i;
			}
		}
		p2 = p3++;
		l2 = l3++;
	}

	for(int i = 1; i < max_row; ++i) {
		// first column pixel can only consider top value
		if(unlikely(*p3 > 0)) {
			if(*p1 == 0) {
				// new component
				++label;
				*l3 = label;
				cv::Point region_start(0,i);
				tl_[label] = region_start;
				br_[label] = region_start;
			} else {
				uint16_t top_label = *l1;	// background for negative rows
				top_label = L_[top_label];
				*l3 = top_label;

				++size_[top_label];
				// top and j have the same x value => no br.x change
				br_[top_label].y = i;	// since this is currently the max row, it is always max
			}
		}
		p0 = p1++;
		p2 = p3++;
		l0 = l1++;
		l2 = l3++;
		// all subsequent pixels in the same row:
		for(int j = 1; j < max_col; ++j) {
			if(unlikely(*p3 > 0)) {	// not background
				if(*p2 > 0) {
					// connected to left label
					uint16_t left_label = *l2;
					if(*p1 > 0) {
						uint16_t top_label = *l1;
						// also connected to top label => merge
						if(left_label != top_label) {
							// compress top_label
							uint16_t init_label = top_label;
							int count = -1;	// count starts at -1, because we only need to modify L in case L != L[L[i]] and not just label != L[i]
							while(top_label != L_[top_label]) {
								top_label = L_[top_label];
								++count;
							}
							for(int i = 0; unlikely(i < count); ++i) {
								uint16_t tmp = L_[init_label];
								L_[init_label] = top_label;
								init_label = tmp;
							}

							// left label cannot be compressed because its values either originate from the already compressed top row or
							// a new (greater) label is introduced in this row and merged with the (smaller) top immediately
							if(unlikely(left_label != L_[left_label])) {
								left_label = L_[left_label];
							}

							if(top_label < left_label) {
								*l3 = top_label;
								L_[left_label] = top_label;

								++size_[top_label];
							} else {
								*l3 = left_label;
								L_[top_label] = left_label;

								++size_[left_label];
							}
						} else {
							// compress label (both are equal)
							uint16_t init_label = top_label;
							int count = -1;
							while(unlikely(top_label != L_[top_label])) {
								top_label = L_[top_label];
								++count;
							}

							for(int i = 0; unlikely(i < count); ++i) {
								uint16_t tmp = L_[init_label];
								L_[init_label] = top_label;
								init_label = tmp;
							}

							*l3 = top_label;
							++size_[top_label];
						}
					} else {
						left_label = L_[left_label];
						*l3 = left_label;

						++size_[left_label];
						if(likely(j > br_[left_label].x)) {	// we might have a bigger x value in the upper rows
							br_[left_label].x = j;
						}
					}
				} else if(*p1 > 0) {
					// only connected to top label
					uint16_t top_label = L_[*l1];
					*l3 = top_label;

					++size_[top_label];
					br_[top_label].y = i;	// since this is currently the max row, it is always max
				} else if(likely(*p0 == 0)) {
					// not connected at all => new element, new label
					++label;
					*l3 = label;

					cv::Point region_start(j,i);
					tl_[label] = region_start;
					br_[label] = region_start;
				} else { // only topleft_label > 0
					// add diagonal connection (top_left to current pixel only)
					uint16_t topleft_label = L_[*l0];
					*l3 = topleft_label;

					++size_[topleft_label];
					if(j > br_[topleft_label].x) {	// we might have a bigger x value in the upper rows
						br_[topleft_label].x = j;
					}
					br_[topleft_label].y = i;	// we definitely have a bigger y-value
				}
			} else {
				// both are connected but not via our current pixel
				if(unlikely(*p1 > 0 && *p2 > 0)) {
					uint16_t top_label = *l1;	// background for negative rows
					uint16_t left_label = *l2;	// background for negative cols
					if(left_label != top_label) {
						// compress top_label
						uint16_t init_label = top_label;
						int count = -1;
						while(top_label != L_[top_label]) {
							top_label = L_[top_label];
							++count;
						}
						for(int i = 0; unlikely(i < count); ++i) {
							uint16_t tmp = L_[init_label];
							L_[init_label] = top_label;
							init_label = tmp;
						}

						// left label cannot be compressed because its values either originate from the already compressed top row or
						// a new (greater) label is introduced in this row and merged with the (smaller) top immediately
						if(unlikely(left_label != L_[left_label])) {
							left_label = L_[left_label];
						}

						if(top_label < left_label) {
							L_[left_label] = top_label;
						} else {
							L_[top_label] = left_label;
						}
					} else {
						// compress label (both are equal)
						int init_label = top_label;
						int count = -1;
						while(unlikely(top_label != L_[top_label])) {
							top_label = L_[top_label];
							++count;
						}
						for(int i = 0; unlikely(i < count); ++i) {
							uint16_t tmp = L_[init_label];
							L_[init_label] = top_label;
							init_label = tmp;
						}
					}
				}
			}
			p0 = p1++;
			p2 = p3++;
			l0 = l1++;
			l2 = l3++;
		}
	}

	size_t num_labels = label+1;
	// do recursive merging
	for(size_t i = 1; i < num_labels; ++i) {
		if(L_[i] < i) {
			uint16_t base_label = L_[i];
			while(unlikely(base_label != L_[base_label])) { // merge
				base_label = L_[base_label];
			}
			L_[i] = base_label;

			size_[base_label] += size_[i];
			if(tl_[i].x < tl_[base_label].x)
				tl_[base_label].x = tl_[i].x;
			if(unlikely(br_[i].x > br_[base_label].x))
				br_[base_label].x = br_[i].x;
			if(unlikely(br_[i].y > br_[base_label].y))
				br_[base_label].y = br_[i].y;
		}
	}

	// buffer the perimeter already computed
	int min_perimeter = min_length_estimate << 2;
	int perimeters[num_labels];
	uint16_t num_component = 0;
	// determine lookup table (required) for the actual components
	for(size_t i = 1; i < num_labels; ++i) {
		if(L_[i] == i) {
			uint16_t base_label = L_[i];
			L_[i] = 0;
			if(unlikely(size_[base_label] >= min_perimeter)) { // unlikely
				int component_min = br_[base_label].x - tl_[base_label].x;
				int component_max = br_[base_label].y - tl_[base_label].y;
				if(component_min > component_max) {
					int temp = component_min;
					component_min = component_max;
					component_max = temp;
				}

				if(unlikely(component_max <= max_length_estimate && component_min >= min_length_estimate && (float)component_min > opposing_length_ratio_*(float)component_max)) {
					++num_component;
					L_[i] = num_component;
					perimeters[num_component] = (component_min + component_max) << 1;
				}
			}
		} else { // was merged, to a lower number, so use that number
			L_[i] = L_[L_[i]];
		}
	}

	// reset size to original status
	memcpy(&size_, &size_template_, num_labels*sizeof(int));

	//cout << "components: " << num_component << endl;
	++num_component;

	// now apply the lookup table (required)
	CvMemStorage *storage = cvCreateMemStorage();
	CvSeq* component_map[num_component];
	for(size_t i = 1; i < num_component; ++i) {
		component_map[i] = cvCreateSeq(CV_SEQ_FLAG_SIMPLE | CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);
	}

	// reduce the pixels of a component to the min and max parts in each row
	uint16_t *pixel_labels_row = (uint16_t*)pixel_labels.data;
	//image_row = image.data;
	int max_entries = min((int)num_component, image.cols);
	int min_coord[max_entries];
	int max_coord[max_entries];
	// fast initialize to -1
	memset(min_coord, -1, max_entries*sizeof(int));
	memset(max_coord, -1, max_entries*sizeof(int));

	int handled_components[max_entries];
	int handle_index = 0;
	for(int i = 0; i < max_row; ++i) {
		int j = 0;
		for( ; j < max_col-1; j += 2) {
			int v0 = pixel_labels_row[0];
			int x0 = L_[v0];
			if(unlikely(x0 > 0)) {
				if(min_coord[x0] >= 0) {
					max_coord[x0] = j;
				} else {
					min_coord[x0] = j;
					handled_components[handle_index] = x0;
					++handle_index;
				}
			}

			int v1 = pixel_labels_row[1];
			int x1 = L_[v1];
			if(unlikely(x1 > 0)) {
				if(min_coord[x1] >= 0) {
					max_coord[x1] = j+1;
				} else {
					min_coord[x1] = j+1;
					handled_components[handle_index] = x1;
					++handle_index;
				}
				/*image_row[j] = 128;
			} else {
				if(image_row[j] > 0) {
					image_row[j] = 32;
				}*/
			}

			pixel_labels_row += 2;
		}
		for( ; j < max_col; ++j) {
			int v = pixel_labels_row[0];
			int x = L_[v];
			if(unlikely(x > 0)) {
				if(min_coord[x] >= 0) {
					max_coord[x] = j;
				} else {
					min_coord[x] = j;
					handled_components[handle_index] = x;
					++handle_index;
				}
			}

			++pixel_labels_row;
		}

		for(int j = 0; j < handle_index; ++j) {
			int label = handled_components[j];
			CvPoint p;
			p.x = min_coord[label]; p.y = i;
			min_coord[label] = -1;
			cvSeqPush(component_map[label], &p);
			if(max_coord[label] >= 0) {
				p.x = max_coord[label];
				max_coord[label] = -1;
				cvSeqPush(component_map[label], &p);
			}
		}

		handle_index = 0;	// reset index
		//image_row += image.step;
	}

	// reset the labels
	memcpy(&L_, &label_template_, num_labels*sizeof(uint16_t));

	//size_t num_candidates = 0;
	for(size_t i = 1; i < num_component; ++i) {
		vector<CvPoint*> hull_vector(2*component_map[i]->total);
		convexHullMC(component_map[i], hull_vector);

		// transform hull into CvSeq
		CvSeqWriter writer;
		cvStartWriteSeq(CV_SEQ_FLAG_SIMPLE | CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage, &writer);
		for(size_t j = 0; j < hull_vector.size(); ++j) {
			CV_WRITE_SEQ_ELEM(*hull_vector[j], writer);
		}
		CvSeq *hull = cvEndWriteSeq(&writer);
		CvSeq *hull_ordered = cvConvexHull2(hull, storage, CV_CLOCKWISE, 1);
		/*cout << hull->total << "," << hull2->total << endl;
		for(int j = 0; j < hull->total; ++j) {
			CvPoint *p = (CvPoint*)cvGetSeqElem(hull, j);
			cout << p->x << "," << p->y << ";";
		}
		cout << endl;
		for(int j = 0; j < hull2->total; ++j) {
			CvPoint *p = (CvPoint*)cvGetSeqElem(hull2, j);
			cout << p->x << "," << p->y << ";";
		}
		cout << endl;*/

		//float length = contourLength(hull, true);
		CvSeq *approx_curve = cvApproxPoly(hull_ordered, sizeof(CvContour), storage, CV_POLY_APPROX_DP, perimeters[i]*0.05);
		if(approx_curve->total == 4) {
			//++num_candidates;
			// transform to proper format
			vector<cv::Point> square(4);
			square[0] = *(CvPoint*)cvGetSeqElem(approx_curve, 0);
			square[1] = *(CvPoint*)cvGetSeqElem(approx_curve, 3);
			square[2] = *(CvPoint*)cvGetSeqElem(approx_curve, 2);
			square[3] = *(CvPoint*)cvGetSeqElem(approx_curve, 1);
			candidates.push_back(square);

			/*for(size_t j = 0; j < 4; ++j) {
				line(image, square[j], square[MOD4((j+1))], cv::Scalar(255), 1);
			}*/
		}/* else if(approx_curve->total == 5) {
			// correct 5 corner polygon to quadrilateral
			// determine minimal side length and incident points
			int min_length = INT_MAX;
			int min_index = 0;
			CvPoint *p = (CvPoint*)cvGetSeqElem(approx_curve, 0);
			CvPoint *p_next;
			for(int i = 0; i < 5; ++i, p = p_next) {
				p_next = (CvPoint*)cvGetSeqElem(approx_curve, (i+1)%5);
				int dx = p_next->x - p->x;
				int dy = p_next->y - p->y;
				int length = dx*dx + dy*dy;
				if(length < min_length) {
					min_length = length;
					min_index = i;
				}
			}

			// determine replacement for the two points i and (i+1)%4
			// get the points involved
			cv::Point prev(*(CvPoint*)cvGetSeqElem(approx_curve, (min_index+4)%5));
			cv::Point succ(*(CvPoint*)cvGetSeqElem(approx_curve, (min_index+2)%5));
			CvPoint *p_min1 = (CvPoint*)cvGetSeqElem(approx_curve, (min_index));
			CvPoint *p_min2 = (CvPoint*)cvGetSeqElem(approx_curve, ((min_index+1)%5));

			// determine lines through the points
			cv::Vec4f line1(p_min1->x-prev.x,p_min1->y-prev.y,p_min1->x,p_min1->y);
			cv::Vec4f line2(p_min2->x-succ.x,p_min2->y-succ.y,p_min2->x,p_min2->y);
			// and their crossing
			cv::Point2f crossing(getCrossing(line1, line2));

			// add the cross point into the vector instead of p_min1 and p_min2
			vector<cv::Point> square;
			square.reserve(4);
			square.push_back(cv::Point(ROUND(crossing.x), ROUND(crossing.y)));
			square.push_back(succ);
			square.push_back(cv::Point(*(CvPoint*)cvGetSeqElem(approx_curve, (min_index+3)%5)));
			square.push_back(prev);
			candidates.push_back(square);

			line(image, cv::Point(*p_min1), cv::Point(*p_min2), cv::Scalar(255), 2);
			line(image, square[0], square[1], cv::Scalar(255), 1);
			line(image, square[1], square[2], cv::Scalar(255), 1);
			line(image, square[2], square[3], cv::Scalar(255), 1);
			line(image, square[3], square[0], cv::Scalar(255), 1);
		}*/
	}
	//cout << num_candidates << endl;

	cvReleaseMemStorage(&storage);
}

void ArucoMarkerDetector::init_static() {
	for(size_t i = 0; i < 65536; ++i) {
		size_template_[i] = 1;
		label_template_[i] = i;
	}
}

} /* namespace fml */
