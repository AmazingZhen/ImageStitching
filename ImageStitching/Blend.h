#ifndef _BLEND_H_
#define _BLEND_H_

#include <iostream>
#include "Warping.h"

using namespace std;

CImg<unsigned char> blendTwoImages(const CImg<unsigned char> &a, const CImg<unsigned char> &b);

#endif