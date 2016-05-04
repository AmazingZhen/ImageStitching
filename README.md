# ImageStitching
A solution for images stitchinging based on Sift, kd-tree, RANSAC and Multi-band Blending.  
Based on [CImg Library](http://cimg.eu/) and [Vlfeat library](http://www.vlfeat.org/).

## Input
- A set of images, supporting disordered images.
- Two sample input sets, "dataset1" and "dataset2".

## Output
- A stitched image.
- Output images are in the folder "res".

## Algorithmic process
- Image registration
  + Cylinder projection
  + Feature extraction
  + Feature matching
  + RANSAC and least squares for homography
- Image blending
  + Image wrapping
  + Multi-band blending

## Areas for improvement
- Compensate explosion error.
