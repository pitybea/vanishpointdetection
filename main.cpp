#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Traj.h"
#include <iostream>
#include <thread>
using namespace std;
#include <direct.h>
#include <Windows.h>

using namespace traj;
using namespace cv;
void testsimple(const Mat& fir,const Mat& sec)
{

		Mat src_gray;
		cvtColor( fir, src_gray, CV_BGR2GRAY );

		vector<Point2f> fea1(kptDet_maxCorners),fea2(kptDet_maxCorners),fea3(kptDet_maxCorners);
	//	fea1.resize(kptDet_maxCorners);
		goodFeaturesToTrack( src_gray,
					fea1,
					kptDet_maxCorners,
					kptDet_qualityLevel,
					kptDet_minDistance,
					Mat(),
					kptDet_blockSize,
					kptDet_useHarrisDetector,
					kptDet_k );

		vector<uchar> status;
		vector<float> err;
		TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, kptTrack_iter, kptTrack_epsin);
		calcOpticalFlowPyrLK(fir,sec,
				fea1,fea2,status,err,
				Size(kptTrack_winsize,kptTrack_winsize),
				kptTrack_maxlevel, termcrit, 0, 0.001);

		for(size_t ti=0;ti<status.size();++ti)
		{
			if(status[ti]!='\1')
				fea2[ti]=Point2f(-1.0,-1.0);
		}

		cvtColor( sec, src_gray, CV_BGR2GRAY );

		goodFeaturesToTrack( src_gray,
					fea3,
					kptDet_maxCorners,
					kptDet_qualityLevel,
					kptDet_minDistance,
					Mat(),
					kptDet_blockSize,
					kptDet_useHarrisDetector,
					kptDet_k );

		Mat copy=sec.clone();

		drawPoints(copy,fea2,2);
		drawPoints(copy,fea3,7);

		imshow("whatever",copy);
		waitKey();
}
	auto sfuc(const vector<Point2f>& inp)->vector<Point2f>
	{
		vector<Point2f> rslt;
		rslt.reserve(inp.size());

		int in=0;
		while(inp[in].x<0.01)
			++in;
		while((in<inp.size())&&(inp[in].x>0.01))
		{	rslt.push_back(inp[in]);
			++in;
		}
		return rslt;
	}
int main(int argc, char* argv[])
{
	//_chdir("E:\\SphericalVideoStabilization\\testdata");

	auto flnms=fileIOclass::InVectorString("imgs.txt");
	vector<Mat> imgs(flnms.size());
	for (size_t i = 0; i < flnms.size(); i++)
	{
		imgs[i]=imread(flnms[i]);
	}

	auto trajs=incrementalTrajectoryDetect(imgs);

	vector<bool> lbs(trajs.size(),false);
	for (int i = 0; i < trajs.size(); i++)
	{
		if(trajs[i][0].x>0.1)
			lbs[i]=true;
	}

	Mat img=imgs[0];

	vector<vector<Point2f> > effect_trajs(trajs.size());


	for (int i = 0; i < trajs.size(); i++)
	{
		effect_trajs[i]=sfuc(trajs[i]);
	}

	drawTrajs_ok(img,effect_trajs);
	imshow("whatever",img);

	FILE* fp;
	fopen_s(&fp,"trajs.txt","w");
	fprintf(fp,"%d %d %d %d\n",2*imgs.size(),trajs.size(),imgs[0].cols,imgs[0].rows);
	for (int i = 0; i < trajs[0].size(); i++)
	{
		for (int j = 0; j < trajs.size(); j++)
		{
			fprintf(fp,"%lf ",(double)trajs[j][i].x);
		}
		fprintf(fp,"\n");
	}

	for (int i = 0; i < trajs[0].size(); i++)
	{
		for (int j = 0; j < trajs.size(); j++)
		{
			fprintf(fp,"%lf ",(double)trajs[j][i].y);
		}
		fprintf(fp,"\n");
	}

	fclose(fp);
	waitKey();
	//testsimple(imgs[0],imgs[1]);
	
	return 1;
}