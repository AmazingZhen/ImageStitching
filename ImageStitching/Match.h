#ifndef _MATCH_H
#define _MATCH_H

#include <iostream>
#include <map>
#include <vector>
#include <cmath>
#include "CImg.h"

using namespace cimg_library;
using namespace std;

#define NUM_OF_PAIR 4
#define CONFIDENCE 0.99
#define INLINER_RATIO 0.5
#define RANSAC_THRESHOLD 4.0

extern "C" {
#include "vl/generic.h"
#include "vl/sift.h"
#include "vl/kdtree.h"
}

struct point_pair {
	VlSiftKeypoint a;
	VlSiftKeypoint b;
	point_pair(VlSiftKeypoint _a, VlSiftKeypoint _b) {
		a = _a;
		b = _b;
	}
};

struct point_pairs {
	vector<point_pair> pairs;
	int src;
	int dst;
	point_pairs(const vector<point_pair> &_pairs, int s, int d) {
		pairs = _pairs;
		src = s;
		dst = d;
	}
};

using namespace std;

struct Parameters {
	float c1;
	float c2;
	float c3;
	float c4;
	float c5;
	float c6;
	float c7;
	float c8;
	Parameters(float _c1, float _c2, float _c3, float _c4, float _c5, float _c6, float _c7, float _c8) {
		c1 = _c1;
		c2 = _c2;
		c3 = _c3;
		c4 = _c4;
		c5 = _c5;
		c6 = _c6;
		c7 = _c7;
		c8 = _c8;
	}
	void print() {
		cout << c1 << " " << c2 << " " << c3 << " " << c4 << endl;
		cout << c5 << " " << c6 << " " << c7 << " " << c8 << endl;
	}
};

float getXAfterWarping(float x, float y, Parameters H);
float getYAfterWarping(float x, float y, Parameters H);

vector<point_pair> getPointPairsFromFeature(const map<vector<float>, VlSiftKeypoint> &feature_a, const map<vector<float>, VlSiftKeypoint> &feature_b);
Parameters getHomographyFromPoingPairs(const vector<point_pair> &pairs);
Parameters RANSAC(const vector<point_pair> &pairs);

#endif