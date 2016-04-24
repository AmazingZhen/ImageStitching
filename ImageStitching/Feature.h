#ifndef _FEATURE_H_
#define _FEATURE_H_

#include <iostream>
#include <map>
#include <vector>
#include "CImg.h"

#define RESIZE_WIDTH 500.0

using namespace cimg_library;
using namespace std;

extern "C" {
	#include "vl/generic.h"
	#include "vl/sift.h"
}

void getFeatureFromImage(const CImg<unsigned char> &src, map<vector<float>, VlSiftKeypoint>& features);

#endif