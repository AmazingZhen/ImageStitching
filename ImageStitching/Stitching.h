#ifndef _STITCHING_H_
#define _STITCHING_H_

#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "Blend.h"

#define MAX_STITCHING_NUM 20

CImg<unsigned char> get_gray_image(const CImg<unsigned char> &srcImg);
CImg<unsigned char> stitching(vector<CImg<unsigned char>> &src_imgs);

#endif