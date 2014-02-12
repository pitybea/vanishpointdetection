#include "ttsetting.inl"
#include "settings.inl"
#include <vector>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <random>
#include <time.h>
using namespace std;// dirty and quick
using namespace cv;//dirty and quick


void SampleExamples(const Mat& img, vector<Point>& tranposs,vector<Point>& testposs,
					vector<Mat>& trainimgs, vector<Mat>& testimgs);