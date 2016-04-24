#include "stdafx.h"
#include "Blend.h"

CImg<unsigned char> blendTwoImages(const CImg<unsigned char> &a, const CImg<unsigned char> &b) {
	assert(a.width() == b.width() && a.height() == b.height());

	CImg<unsigned char> res(a.width(), a.height(), 1, a.spectrum(), 0);

	for (int i = 0; i < res.width(); i++) {
		for (int j = 0; j < res.height(); j++) {
			for (int k = 0; k < res.spectrum(); k++) {
				if (a(i, j, k) == 0 && b(i, j, k) != 0) {
					res(i, j, k) = b(i, j, k);
				}
				else if (a(i, j, k) != 0 && b(i, j, k) == 0) {
					res(i, j, k) = a(i, j, k);
				}
				else {
					res(i, j, k) = a(i, j, k) * 0.5 + b(i, j, k) * 0.5;
				}
			}
		}
	}

	return res;
}
