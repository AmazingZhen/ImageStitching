// ImageStitching.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include "CImg.h"
#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "FileReading.h"
#include "Stitching.h"

#define FILE_FOLDER "dataset2\\"

using namespace cimg_library;
using namespace std;


void draw_point(CImg<unsigned char> &img, int x, int y, double circle) {
	assert(x >= 0 && x < img.width() && y >= 0 && y < img.height());
	int width = img.width();
	int height = img.height();
	for (int i = x - circle; i <= x + circle; i++) {
		for (int j = y - circle; j <= y + circle; j++) {
			if (i < 0 || i >= img.width() || j < 0 || j >= img.height())
				continue;

			if (abs(sqrt((i - x)*(i - x) + (j - y)*(j - y)) - circle) < 0.5) {
				img(i, j, 0, 0) = 0xff;
			}
		}
	}
}

void draw_features(vector<CImg<unsigned char>> src_imgs, vector<map<vector<float>, VlSiftKeypoint>> features) {
	assert(src_imgs.size() == features.size());

	for (int i = 0; i < src_imgs.size(); i++) {
		for (int j = 0; j < src_imgs.size(); j++) {
			if (i == j)
				continue;

			vector<point_pair> a = getPointPairsFromFeature(features[i], features[j]);
			cout << i<< ", " << j << " " << a.size() << endl;

			//CImg<unsigned char> left(src_imgs[i]);
			//CImg<unsigned char> right(src_imgs[j]);

			//for (int k = 0; k < a.size(); k++) {
			//	draw_point(left, a[k].a.ix, a[k].a.iy, 5);
			//	draw_point(right, a[k].b.ix, a[k].b.iy, 5);
			//}

			//(left, right).display();
		}
	}
}

int main(int argc, char **argv) {
	string file_folder(FILE_FOLDER);
	vector<string> image_files;

	getAllFiles(file_folder, image_files);

	vector<CImg<unsigned char>> src_imgs(image_files.size());
	vector<map<vector<float>, VlSiftKeypoint>> features(image_files.size());

	for (int i = 0; i < image_files.size(); i++) {
		src_imgs[i] = CImg<float>(image_files[i].c_str());
		/*CImg<unsigned char> gray = get_gray_image(src_imgs[i]);
		getFeatureFromImage(gray, features[i]);
		cout << i << endl;*/
	}

	stitching(src_imgs);

	/*Parameters H1 = RANSAC(getPointPairsFromFeature(features[1], features[0]));


	CImg<unsigned char> res = warpingImageByHomography(src_imgs[0], H);
	res.display();
	res.save("dataset1\\a.bmp");*/
	
	
	return 0;
}



