// ImageStitching.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include <iostream>
#include <cassert>
#include <cmath>
#include "CImg.h"
#include "Projection.h"
#include "Feature.h"
#include "Match.h"
#include "Warping.h"
#include "FileReading.h"
#include "Stitching.h"

#define FILE_FOLDER "dataset2\\"

using namespace cimg_library;
using namespace std;

int main(int argc, char **argv) {

	string file_folder(FILE_FOLDER);
	vector<string> image_files;

	getAllFiles(file_folder, image_files);

	vector<CImg<unsigned char>> src_imgs(image_files.size());
	vector<map<vector<float>, VlSiftKeypoint>> features(image_files.size());

	for (int i = 0; i < image_files.size(); i++) {
		src_imgs[i] = CImg<unsigned char>(image_files[i].c_str());
	}

	CImg<unsigned char> res = stitching(src_imgs);
	res.display();
	res.save("res/pano.jpg");
	
	return 0;
}



