#include "stdafx.h"
#include "Feature.h"

void getFeatureFromImage(const CImg<unsigned char> &input, map<vector<float>, VlSiftKeypoint>& features) {
	assert(input.spectrum() == 1);

	CImg<unsigned char> src(input);

	float resize_factor = RESIZE_WIDTH / input.width();

	if (resize_factor >= 1) {
		resize_factor = 1;
	}
	else {
		src.resize(src.width() * resize_factor, src.height() * resize_factor, src.depth(), src.spectrum(), 3);
	}

	vl_sift_pix *imageData = new vl_sift_pix[src.width()*src.height()];

	for (int i = 0; i < src.width(); i++) {
		for (int j = 0; j < src.height(); j++) {
			imageData[j*src.width() + i] = src(i, j, 0);
		}
	}

	// Initialize a SIFT filter object.
	int noctaves = 4, nlevels = 2, o_min = 0;
	VlSiftFilt *siftFilt = NULL;
	siftFilt = vl_sift_new(src.width(), src.height(), noctaves, nlevels, o_min);

	// Compute the first octave of the DOG scale space.
	if (vl_sift_process_first_octave(siftFilt, imageData) != VL_ERR_EOF) {
		while (true) {
			// Run the SIFT detector to get the keypoints.
			vl_sift_detect(siftFilt);

			VlSiftKeypoint *pKeyPoint = siftFilt->keys;
			// For each keypoint:
			for (int i = 0; i < siftFilt->nkeys; i++) {
				VlSiftKeypoint tempKeyPoint = *pKeyPoint;

				// Get the keypoint orientation(s).
				double angles[4];
				int angleCount = vl_sift_calc_keypoint_orientations(siftFilt, angles, &tempKeyPoint);

				// For each orientation:
				for (int j = 0; j < angleCount; j++) {
					double tempAngle = angles[j];
					vl_sift_pix descriptors[128];

					// Get the keypoint descriptor
					vl_sift_calc_keypoint_descriptor(siftFilt, descriptors, &tempKeyPoint, tempAngle);

					vector<float> des;
					int k = 0;
					while (k < 128) {
						des.push_back(descriptors[k]);
						k++;
					}

					// http://stackoverflow.com/questions/28606011/vlfeat-kdtree-setup-and-query

					tempKeyPoint.x /= resize_factor;
					tempKeyPoint.y /= resize_factor;
					tempKeyPoint.ix = tempKeyPoint.x;
					tempKeyPoint.iy = tempKeyPoint.y;

					features.insert(pair<vector<float>, VlSiftKeypoint>(des, tempKeyPoint));
				}

				pKeyPoint++;
			}
			if (vl_sift_process_next_octave(siftFilt) == VL_ERR_EOF) {
				break;
			}
		}
	}

	vl_sift_delete(siftFilt);

	delete[] imageData;
	imageData = NULL;

}