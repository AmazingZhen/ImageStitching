#ifndef _BLEND_H_
#define _BLEND_H_

#include <iostream>
#include "CImg.h"
#include <cassert>

using namespace std;
using namespace cimg_library;

CImg<unsigned char> blendTwoImages(const CImg<unsigned char> &a, const CImg<unsigned char> &b);

#endif