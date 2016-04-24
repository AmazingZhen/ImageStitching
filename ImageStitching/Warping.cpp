#include "stdafx.h"
#include "Warping.h"

int getMaxXAfterWarping(const CImg<unsigned char> &src, Parameters H) {
	int max_x = getXAfterWarping(0, 0, H);

	if (getXAfterWarping(src.width() - 1, 0, H) > max_x) {
		max_x = getXAfterWarping(src.width() - 1, 0, H);
	}
	if (getXAfterWarping(0, src.height() - 1, H) > max_x) {
		max_x = getXAfterWarping(0, src.height() - 1, H);
	}
	if (getXAfterWarping(src.width() - 1, src.height() - 1, H) > max_x) {
		max_x = getXAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return max_x;
}

int getMinXAfterWarping(const CImg<unsigned char> &src, Parameters H) {
	int min_x = getXAfterWarping(0, 0, H);

	if (getXAfterWarping(src.width() - 1, 0, H) < min_x) {
		min_x = getXAfterWarping(src.width() - 1, 0, H);
	}
	if (getXAfterWarping(0, src.height() - 1, H) < min_x) {
		min_x = getXAfterWarping(0, src.height() - 1, H);
	}
	if (getXAfterWarping(src.width() - 1, src.height() - 1, H) < min_x) {
		min_x = getXAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return min_x;
}

int getMaxYAferWarping(const CImg<unsigned char> &src, Parameters H) {
	int max_y = getYAfterWarping(0, 0, H);

	if (getYAfterWarping(src.width() - 1, 0, H) > max_y) {
		max_y = getYAfterWarping(src.width() - 1, 0, H);
	}
	if (getYAfterWarping(0, src.height() - 1, H) > max_y) {
		max_y = getYAfterWarping(0, src.height() - 1, H);
	}
	if (getYAfterWarping(src.width() - 1, src.height() - 1, H) > max_y) {
		max_y = getYAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return max_y;
}

int getMinYAfterWarping(const CImg<unsigned char> &src, Parameters H) {
	int min_y = getYAfterWarping(0, 0, H);

	if (getYAfterWarping(src.width() - 1, 0, H) < min_y) {
		min_y = getYAfterWarping(src.width() - 1, 0, H);
	}
	if (getYAfterWarping(0, src.height() - 1, H) < min_y) {
		min_y = getYAfterWarping(0, src.height() - 1, H);
	}
	if (getYAfterWarping(src.width() - 1, src.height() - 1, H) < min_y) {
		min_y = getYAfterWarping(src.width() - 1, src.height() - 1, H);
	}

	return min_y;
}

int getWidthAfterWarping(const CImg<unsigned char> &src, Parameters H) {
	int max = getMaxXAfterWarping(src, H);
	int min = getMinXAfterWarping(src, H);

	return max - min;
}

int getHeightAfterWarping(const CImg<unsigned char> &src, Parameters H) {
	int max = getMaxYAferWarping(src, H);
	int min = getMinYAfterWarping(src, H);

	return max - min;
}

void warpingImageByHomography(const CImg<unsigned char> &src, CImg<unsigned char> &dst, Parameters H, int offset_x, int offset_y) {
	for (int dst_x = 0; dst_x < dst.width(); dst_x++) {
		for (int dst_y = 0; dst_y < dst.height(); dst_y++) {
			int src_x = getXAfterWarping(dst_x + offset_x, dst_y + offset_y, H);
			int src_y = getYAfterWarping(dst_x + offset_x, dst_y + offset_y, H);

			if (src_x >= 0 && src_x < src.width() && src_y >= 0 && src_y < src.height()) {
				for (int k = 0; k < src.spectrum(); k++) {
					dst(dst_x, dst_y, k) = bilinear_interpolation(src, src_x, src_y, k);
				}
			}
		}
	}
	
	//for (int i = 0; i < src.width(); i++) {
	//	for (int j = 0; j < src.height(); j++) {
	//		int x;
	//		int y;
	//		if (min_x < 0) {
	//			x = getXAfterWarping(i, j, H) - min_y;
	//		}
	//		else {
	//			x = getXAfterWarping(i, j, H);
	//		}

	//		if (min_y < 0) {
	//			y = getYAfterWarping(i, j, H) - min_y;
	//		}
	//		else {
	//			y = getYAfterWarping(i, j, H);
	//		}

	//		for (int k = 0; k < src.spectrum(); k++) {
	//			res(x, y, k) = src(i, j, k);
	//		}
	//	}
	//}
}

void movingImageByOffset(const CImg<unsigned char> &src, CImg<unsigned char> &dst, int offset_x, int offset_y) {
	for (int dst_x = 0; dst_x < dst.width(); dst_x++) {
		for (int dst_y = 0; dst_y < dst.height(); dst_y++) {
			int src_x = dst_x + offset_x;
			int src_y = dst_y + offset_y;

			if (src_x >= 0 && src_x < src.width() && src_y >= 0 && src_y < src.height()) {
				for (int k = 0; k < src.spectrum(); k++) {
					dst(dst_x, dst_y, k) = src(src_x, src_y, k);
				}
			}
		}
	}
}