#pragma once

#include <opencv2/core.hpp>
#include "stdafx.h"
#include<fstream>

using namespace cv;
using namespace std;


void SOS(Mat left, Mat right, int maxDisparity, Mat& disparityMap, Mat& depthMap, double focal_length, double baseline);
float computeScore(Mat& disparityMap, Mat& groundTruth);
Mat generateErrorVisualization(Mat& disparityMap, Mat& gt);