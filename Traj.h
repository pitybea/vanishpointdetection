
#pragma once 
#include "../FileIO/FileInOut.h"
//#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "TrajSettings.inl"
//using namespace vanish;
using namespace cv; //quick and dirty
#include <vector>
#include <tuple>

auto incrementalTrajectoryDetect(const vector<Mat>& imgs)-> vector<vector<Point2f> >;