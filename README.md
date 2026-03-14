# **Image Panorama Generator**
A MATLAB implementation of a full computer vision pipeline for automatic panorama stitching from overlapping image pairs.
Pipeline

- FAST corner detection: Custom implementation using 16-point Bresenham circle tests with configurable threshold t and contiguous pixel count n. Non-maximum suppression via local score maximization
- Harris cornerness scoring: Filters FAST corners using second-moment matrix estimation with Sobel gradients and Gaussian weighting, improving corner quality and reducing false positives
- Feature description & matching: ORB descriptors extracted at Harris-filtered keypoints, matched across image pairs using MATLAB's matchFeatures
- RANSAC homography: Projective transform estimated with estgeotform2d, robust to outlier matches, used to warp and blend the second image into a shared panorama canvas

# **How to Run**
Requires MATLAB with the Image Processing Toolbox and Computer Vision Toolbox.

1. Clone the repo and open main.m in MATLAB
2. Replace the hardcoded image filenames (e.g. "S1-im1.png", "S2-im1.jpg") near the top of generateImages() and at the bottom of the script with your own overlapping image pairs
3. Run the script — output panoramas are saved as S1-panorama.png, S2-panorama.png, etc.

# **Implementation Notes**

- FAST uses n=12 contiguous pixels and threshold t=0.1, matching the OpenCV specification
- The 16-point circle is defined via hand-coded impulse kernels based on the OpenCV FAST documentation
- Harris threshold is set to 0.001 — lower this if too few corners are detected on your images
- Source images are not included as they are the property of others
