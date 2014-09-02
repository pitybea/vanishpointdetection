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

#include <map>


#include "sfm.inl"
using namespace sfm;

using namespace traj;
using namespace cv;


#include "parallelSetting.inl"
using namespace parallel;




int mainsd(int argc,char* argv[])
{
	_chdir("D:\\DATA\\ttwoladybug\\ladybug\\tss");

	string s="ladybug_panoramic_001940.jpg.tsk";

	if(argc>1)
		s=argv[1];

	auto imgNames=fileIOclass::InVectorString(s);

	vector<vector<double> > errors=fileIOclass::InVectorSDouble(s+".tmat.error",9);

	vector<double> sumError(errors.size(),0.0);
	
	for(size_t i=0;i<errors.size();++i)
		for(size_t j=0;j<9;j++)
			sumError[i]+=abs(errors[i][j]);



	vector<vector<int> > f[2],match;
	f[0]=fileIOclass::InVectorSInt(imgNames[0]+".orb",2);
	f[1]=fileIOclass::InVectorSInt(imgNames[1]+".orb",2);
	match=fileIOclass::InVectorSInt(s+".mtch",2);

	vector<vector<bool> > matched(2);
	for(int i=0;i<2;++i)
	matched[i]=vector<bool> (f[i].size(),false);

	Mat imgs[2];
	for(int i=0;i<2;++i)
		imgs[i]=imread(imgNames[i],-1);

	//drawMatches
	//imshow("ok",imgs[0]);

	Size size(imgs[0].cols*2,imgs[0].rows);

	Mat outimage;
	outimage.create(size,CV_MAKETYPE(imgs[0].depth(),3));

	//Mat& reffer=;

	imgs[0].copyTo(outimage(Rect(0,0,size.width/2,size.height)));
	
	//Mat& areffer;

	imgs[1].copyTo(outimage(Rect(size.width/2,0,size.width/2,size.height)));
	
	
	for(size_t i=0;i<match.size();++i)
	{
		int in1=match[i][0];
		int in2=match[i][1];

		matched[0][in1]=true;
		matched[1][in2]=true;

		Point p1=Point(f[0][in1][0],f[0][in1][1]);
		Point p2=Point(f[1][in2][0]+size.width/2,f[1][in2][1]);
		
		Scalar s;
		if(sumError[i]<0.01)
		{
			s=Scalar(0,255,0);
		}
		else
		{
			s=Scalar(0,0,255);
		}
		drawPoint(outimage,p1,2,s);
		drawPoint(outimage,p2,2,s);
		line(outimage,p1,p2,s);
	}

	int ex[2]={0,size.width/2};
	for(int i=0;i<2;++i)
		for(int j=0;j<f[i].size();++j)
		{
			if(!matched[i][j])
			drawPoint(outimage,Point(f[i][j][0]+ex[i],f[i][j][1]),2);
		}

	imwrite(s+".mtch.jpg",outimage);
	imshow("i",outimage);
	//waitKey(0);
	return 0;
}