#ifndef HW1_HPP
#define HW1_HPP

#include <opencv2/opencv.hpp>
using namespace cv;

class ColorFilter {
public:
    ColorFilter();

    void createBinaryMasks(const Mat& img_hsv, Mat& green_mask, Mat& red_mask, Mat& blue_mask);
    void createGaussianMasks(const Mat& img, Mat& green_mask, Mat& red_mask, Mat& blue_mask, int blur_sigma = 5);

private:
    Scalar lower_green, upper_green;
    Scalar lower_red1, upper_red1, lower_red2, upper_red2;
    Scalar lower_blue, upper_blue;
};

#endif // HW1_HPP

