
#pragma once 
#include "../FileIO/FileInOut.h"
//#include <cv.h>
#include "utils.h"
#include "TrajSettings.inl"
//using namespace vanish;
using namespace cv; //quick and dirty
#include <vector>
#include <tuple>

auto incrementalTrajectoryDetect(const vector<Mat>& imgs)-> vector<vector<Point2f> >;

void removeStaticTrajectories(vector<vector<Point2f> > &trajs);