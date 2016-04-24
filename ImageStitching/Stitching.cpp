#include "stdafx.h"
#include "Stitching.h"

bool graph[MAX_STITCHING_NUM][MAX_STITCHING_NUM] = { false };
bool has_stitched[MAX_STITCHING_NUM] = { false };
vector<point_pairs > matching_pairs[MAX_STITCHING_NUM];

CImg<unsigned char> get_gray_image(const CImg<unsigned char> &srcImg) {
	if (srcImg.spectrum() == 1) {
		return srcImg;
	}
	int width = srcImg.width();
	int height = srcImg.height();
	int depth = srcImg.depth();
	CImg<unsigned char> grayImg(width, height, depth, 1);
	float r, g, b, gr;
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			r = srcImg(i, j, 0, 0);
			g = srcImg(i, j, 0, 1);
			b = srcImg(i, j, 0, 2);
			gr = 0.299*(r)+0.587*(g)+0.114*(b);
			grayImg(i, j, 0, 0) = gr;
		}
	}
	return grayImg;
}

CImg<unsigned char> stitching(const vector<CImg<unsigned char>> &src_imgs) {
	vector<map<vector<float>, VlSiftKeypoint>> features(src_imgs.size());

	for (int i = 0; i < src_imgs.size(); i++) {
		CImg<unsigned char> gray = get_gray_image(src_imgs[i]);
		getFeatureFromImage(gray, features[i]);
		cout << i << endl;
	}

	for (int i = 0; i < src_imgs.size(); i++) {
		for (int j = 0; j < src_imgs.size(); j++) {
			if (i == j)
				continue;

			vector<point_pair> pairs = getPointPairsFromFeature(features[i], features[j]);
			if (pairs.size() >= 20) {
				graph[i][j] = true;
				matching_pairs[i].push_back(point_pairs(pairs, i, j));
			}

		}
	}

	for (int src_index = 0; src_index < src_imgs.size(); src_index++) {
		if (has_stitched[src_index] == true)
			continue;

		for (int j = 0; j < matching_pairs[src_index].size(); j++) {
			int dst_index = matching_pairs[src_index][j].dst;

			vector<point_pair> &src_to_dst_pairs = matching_pairs[src_index][j].pairs;
			vector<point_pair> &dst_to_src_pairs = vector<point_pair>();

			for (int k = 0; k < matching_pairs[dst_index].size(); k++) {
				if (matching_pairs[dst_index][k].dst == src_index) {
					dst_to_src_pairs = matching_pairs[dst_index][k].pairs;
					break;
				}
			}

			Parameters forward_H = RANSAC(dst_to_src_pairs);
			Parameters backward_H = RANSAC(src_to_dst_pairs);

			int min_x = getMinXAfterWarping(src_imgs[dst_index], forward_H);
			min_x = (min_x < 0) ? min_x : 0;

			int min_y = getMinYAfterWarping(src_imgs[dst_index], forward_H);
			min_y = (min_y < 0) ? min_y : 0;

			int max_x = getMaxXAfterWarping(src_imgs[dst_index], forward_H);
			max_x = (max_x >= src_imgs[src_index].width()) ? max_x : src_imgs[src_index].width();

			int max_y = getMaxYAferWarping(src_imgs[dst_index], forward_H);
			max_y = (max_y >= src_imgs[src_index].height()) ? max_y : src_imgs[src_index].height();

			int new_width = max_x - min_x;
			int new_height = max_x - min_x;

			CImg<unsigned char> a(new_width, new_height, 1, src_imgs[dst_index].spectrum(), 0);
			CImg<unsigned char> b(new_width, new_height, 1, src_imgs[dst_index].spectrum(), 0);
			warpingImageByHomography(src_imgs[dst_index], a, backward_H, min_x, min_y);
			movingImageByOffset(src_imgs[src_index], b, min_x, min_y);

			(a,b).display();
		}

	}


	// Unfinished.
	return CImg<unsigned char>(1, 1, 1, 1, 0);
}