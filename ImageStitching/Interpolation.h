#ifndef _INTERPOLATION_H_
#define _INTERPOLATION_H_

#include "CImg.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <assert.h>

using namespace cimg_library;
using namespace std;

double sinxx(double value);

template <class T>
T bilinear_interpolation(const CImg<T>& image, double x, double y, int channel) {
    assert(x >= 0 && x < image.width());
    assert(y >= 0 && y < image.height());
    assert(channel <= image.spectrum());

    int x_pos = floor(x);
    double x_u = x - x_pos;
    int xb = (x_pos < image.width() - 1) ? x_pos + 1 : x_pos;

    int y_pos = floor(y);
    double y_v = y - y_pos;
    int yb = (y_pos < image.height() - 1) ? y_pos + 1 : y_pos;

    double P1 = image(x_pos, y_pos, channel) * (1 - x_u) + image(xb, y_pos, channel) * x_u;
    double P2 = image(x_pos, yb, channel) * (1 - x_u) + image(xb, yb, channel) * x_u;

    return P1 * (1 - y_v) + P2 * y_v;
}

template <class T>
T bicubic_interpolation(const CImg<T>& image, double x, double y, int channel) {
    assert(x >= 0 && x < image.width());
    assert(y >= 0 && y < image.height());
    assert(channel <= image.spectrum());

    int x_pos = floor(x);
    double x_u = x - x_pos;
    int y_pos = floor(y);
    double y_v = y - y_pos;

    int xa = (x_pos > 0) ? x_pos - 1 : x_pos;
    int xc = (x_pos < image.width() - 1) ? x_pos + 1 : x_pos;
    int xd = (x_pos < image.width() - 2) ? x_pos + 2 : xc;

    int ya = (y_pos > 0) ? y_pos - 1 : y_pos;
    int yc = (y_pos < image.height() - 1) ? y_pos + 1 : y_pos;
    int yd = (y_pos < image.height() - 2) ? y_pos + 2 : yc;

    CImg<double> A(4, 1, 1, 1, sinxx(x_u + 1), sinxx(x_u), sinxx(x_u - 1), sinxx(x_u - 2));
    CImg<double> C(1, 4, 1, 1, sinxx(y_v + 1), sinxx(y_v), sinxx(y_v - 1), sinxx(y_v - 2));

    CImg<double> B(4, 4, 1, 1, image(xa, ya, channel), image(xa, y_pos, channel), image(xa, yc, channel), image(xa, yd, channel),
        image(x_pos, ya, channel), image(x_pos, y_pos, channel), image(x_pos, yc, channel), image(x_pos, yd, channel),
        image(xc, ya, channel), image(xc, y_pos, channel), image(xc, yc, channel), image(xc, yd, channel),
        image(xd, ya, channel), image(xd, y_pos, channel), image(xd, yc, channel), image(xd, yd, channel)
        );

    CImg<double> res = A*B*C;
    return res(0, 0);
}

#endif