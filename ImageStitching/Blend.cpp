#include "stdafx.h"
#include "Blend.h"
#include <cmath>
#include <vector>

bool isEmpty(const CImg<unsigned char> &img, int x, int y) {
	assert(img.spectrum() == 3);

	return (img(x, y, 0) == 0 && img(x, y, 1) == 0 && img(x, y, 2) == 0);
}

CImg<unsigned char> blendTwoImages(const CImg<unsigned char> &a, const CImg<unsigned char> &b) {
	assert(a.width() == b.width() && a.height() == b.height() && a.spectrum() == b.spectrum());

	// Find the center point of a and overlapping part.
	double sum_a_x = 0;
	double sum_a_y = 0;
	int a_n = 0;
	//double sum_b_x = 0;
	//double sum_b_y = 0;
	//int b_n = 0;
	double sum_overlap_x = 0;
	double sum_overlap_y = 0;
	int overlap_n = 0;
	if (a.width() > a.height()) {
		for (int x = 0; x < a.width(); x++) {
			if (!isEmpty(a, x, a.height() / 2)) {
				sum_a_x += x;
				a_n++;
			}

			//if (!isEmpty(b, x, b.height() / 2)) {
			//	sum_b_x += x;
			//	b_n++;
			//}

			if (!isEmpty(a, x, a.height() / 2) && !isEmpty(b, x, a.height() / 2)) {
				sum_overlap_x += x;
				overlap_n++;
			}
		}
	}
	else {
		for (int y = 0; y < a.height(); y++) {
			if (!isEmpty(a, a.width() / 2, y)) {
				sum_a_y += y;
				a_n++;
			}

			if (!isEmpty(a, a.width() / 2, y) && !isEmpty(b, b.width() / 2, y)) {
				sum_overlap_y += y;
				overlap_n++;
			}
		}
	}

	int min_len = (a.width() < a.height()) ? a.width() : a.height();

	int n_level = floor(log2(min_len));

	vector<CImg<float> > a_pyramid(n_level);
	vector<CImg<float> > b_pyramid(n_level);
	vector<CImg<float> > mask(n_level);

	// Initialize the base.
	a_pyramid[0] = a;
	b_pyramid[0] = b;
	mask[0] = CImg<float>(a.width(), a.height(), 1, 1, 0);

	if (a.width() > a.height()) {
		if (sum_a_x / a_n < sum_overlap_x / overlap_n) {
			for (int x = 0; x < sum_overlap_x / overlap_n; x++) {
				for (int y = 0; y < a.height(); y++) {
					mask[0](x, y) = 1;
				}
			}
		}
		else {
			for (int x = sum_overlap_x / overlap_n + 1; x < a.width(); x++) {
				for (int y = 0; y < a.height(); y++) {
					mask[0](x, y) = 1;
				}
			}
		}
	}
	else {
		if (sum_a_y / a_n < sum_overlap_y / overlap_n) {
			for (int x = 0; x < a.width(); x++) {
				for (int y = 0; y < sum_overlap_y / overlap_n; y++) {
					mask[0](x, y) = 1;
				}
			}
		}
		else {
			for (int x = 0; x < a.width(); x++) {
				for (int y = sum_overlap_y / overlap_n; y < a.height(); y++) {
					mask[0](x, y) = 1;
				}
			}
		}
	}

	// Down sampling a and b, building Gaussian pyramids.
	for (int i = 1; i < n_level; i++) {
		a_pyramid[i] = a_pyramid[i - 1].get_blur(2).get_resize(a_pyramid[i - 1].width() / 2, a_pyramid[i - 1].height() / 2, 1, a_pyramid[i - 1].spectrum(), 3);
		b_pyramid[i] = b_pyramid[i - 1].get_blur(2).get_resize(b_pyramid[i - 1].width() / 2, b_pyramid[i - 1].height() / 2, 1, b_pyramid[i - 1].spectrum(), 3);
		
		mask[i] = mask[i - 1].get_blur(2).get_resize(mask[i - 1].width() / 2, mask[i - 1].height() / 2, 1, mask[i - 1].spectrum(), 3);
	}
	
	// Building Laplacian pyramids.
	for (int i = 0; i < n_level - 1; i++) {
		a_pyramid[i] = a_pyramid[i] - a_pyramid[i + 1].get_resize(a_pyramid[i].width(), a_pyramid[i].height(), 1, a_pyramid[i].spectrum(), 3);
		b_pyramid[i] = b_pyramid[i] - b_pyramid[i + 1].get_resize(b_pyramid[i].width(), b_pyramid[i].height(), 1, b_pyramid[i].spectrum(), 3);
	}

	vector<CImg<float> > blend_pyramid(n_level);

	for (int i = 0; i < n_level; i++) {
		blend_pyramid[i] = CImg<float>(a_pyramid[i].width(), a_pyramid[i].height(), 1, a_pyramid[i].spectrum(), 0);

		for (int x = 0; x < blend_pyramid[i].width(); x++) {
			for (int y = 0; y < blend_pyramid[i].height(); y++) {
				for (int k = 0; k < blend_pyramid[i].spectrum(); k++) {
					blend_pyramid[i](x, y, k) = a_pyramid[i](x, y, k) * mask[i](x, y) + b_pyramid[i](x, y, k) * (1.0 - mask[i](x, y));

				}
			}
		}
	}

	CImg<float> res = blend_pyramid[n_level - 1];
	for (int i = n_level - 2; i >= 0; i--) {
		res.resize(blend_pyramid[i].width(), blend_pyramid[i].height(), 1, blend_pyramid[i].spectrum(), 3);

		for (int x = 0; x < blend_pyramid[i].width(); x++) {
			for (int y = 0; y < blend_pyramid[i].height(); y++) {
				for (int k = 0; k < blend_pyramid[i].spectrum(); k++) {
					float t = res(x, y, k) + blend_pyramid[i](x, y, k);

					if (t > 255) {
						t = 255;
					}
					else if (t < 0) {
						t = 0;
					}

					res(x, y, k) = t;
				}
			}
		}
	}

	return res;
}
