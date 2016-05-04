#ifndef _WARPING_H_
#define _WARPING_H_

#include <iostream>
#include "Match.h"
#include "Interpolation.h"

using namespace std;

float getMaxXAfterWarping(const CImg<unsigned char> &src, Parameters H);
float getMinXAfterWarping(const CImg<unsigned char> &src, Parameters H);
float getMaxYAferWarping(const CImg<unsigned char> &src, Parameters H);
float getMinYAfterWarping(const CImg<unsigned char> &src, Parameters H);

int getWidthAfterWarping(const CImg<unsigned char> &src, Parameters H);
int getHeightAfterWarping(const CImg<unsigned char> &src, Parameters H);

void warpingImageByHomography(const CImg<unsigned char> &src, CImg<unsigned char> &dst, Parameters H, float offset_x, float offset_y);
void movingImageByOffset(const CImg<unsigned char> &src, CImg<unsigned char> &dst, int offset_x, int offset_y);


#endif