#ifndef _WARPING_H_
#define _WARPING_H_

#include <iostream>
#include "Match.h"
#include "Interpolation.h"

using namespace std;

int getMaxXAfterWarping(const CImg<unsigned char> &src, Parameters H);
int getMinXAfterWarping(const CImg<unsigned char> &src, Parameters H);
int getMaxYAferWarping(const CImg<unsigned char> &src, Parameters H);
int getMinYAfterWarping(const CImg<unsigned char> &src, Parameters H);

int getWidthAfterWarping(const CImg<unsigned char> &src, Parameters H);
int getHeightAfterWarping(const CImg<unsigned char> &src, Parameters H);

void warpingImageByHomography(const CImg<unsigned char> &src, CImg<unsigned char> &dst, Parameters H, int offset_x, int offset_y);
void movingImageByOffset(const CImg<unsigned char> &src, CImg<unsigned char> &dst, int offset_x, int offset_y);


#endif