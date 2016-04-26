#ifndef _PROJECTION_H_
#define _PROJECTION_H_

#include "CImg.h"

#define ANGLE 15.0
#define PI 3.1415926

using namespace cimg_library;

CImg<unsigned char> cylinderProjection(const CImg<unsigned char> &src);

#endif