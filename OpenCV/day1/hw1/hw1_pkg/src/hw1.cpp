#include <opencv2/opencv.hpp>
#include <iostream>
#include "hw1_pkg/hw1.hpp"

using namespace cv;
using namespace std;

ColorFilter::ColorFilter() {
    lower_green = Scalar(35, 50, 50);
    upper_green = Scalar(85, 255, 255);

    lower_red1 = Scalar(0, 50, 50);
    upper_red1 = Scalar(10, 255, 255);
    lower_red2 = Scalar(160, 50, 50);
    upper_red2 = Scalar(180, 255, 255);

    lower_blue = Scalar(100, 100, 50);
    upper_blue = Scalar(130, 255, 255);
}

void ColorFilter::createBinaryMasks(const Mat& img_hsv, Mat& green_mask, Mat& red_mask, Mat& blue_mask) {
    Mat red_mask1, red_mask2;

    inRange(img_hsv, lower_green, upper_green, green_mask);
    inRange(img_hsv, lower_red1, upper_red1, red_mask1);
    inRange(img_hsv, lower_red2, upper_red2, red_mask2);
    bitwise_or(red_mask1, red_mask2, red_mask);
    inRange(img_hsv, lower_blue, upper_blue, blue_mask);
}

void ColorFilter::createGaussianMasks(const Mat& img, Mat& green_mask, Mat& red_mask, Mat& blue_mask, int blur_sigma) {
    Mat blurred, img_hsv;
    GaussianBlur(img, blurred, Size(), (double)blur_sigma);
    cvtColor(blurred, img_hsv, COLOR_BGR2HSV);
    createBinaryMasks(img_hsv, green_mask, red_mask, blue_mask);
}

int main() {
    string image_path = "/home/ssm/opencv_ws/day1/hw1/hw1_pkg/hw1.png";
    Mat img = imread(image_path, IMREAD_COLOR);

    if(img.empty()) {
        cout << "Image not found!" << endl;
        return -1;
    }

    Mat img_hsv;
    cvtColor(img, img_hsv, COLOR_BGR2HSV);

    ColorFilter cf;

    Mat green_mask, red_mask, blue_mask;
    cf.createBinaryMasks(img_hsv, green_mask, red_mask, blue_mask);

    imshow("Green Mask (Original)", green_mask);
    imshow("Red Mask (Original)", red_mask);
    imshow("Blue Mask (Original)", blue_mask);

    Mat g_green_mask, g_red_mask, g_blue_mask;
    cf.createGaussianMasks(img, g_green_mask, g_red_mask, g_blue_mask, 5);

    imshow("Green Mask (Gaussian)", g_green_mask);
    imshow("Red Mask (Gaussian)", g_red_mask);
    imshow("Blue Mask (Gaussian)", g_blue_mask);

    waitKey(0);
    return 0;
}
