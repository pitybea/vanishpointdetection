#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"
#include <iostream>
using namespace std;

static string cities[3]={"ny","paris","kyoto"};

void vanishiDetectForCity(string s)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");

	vector<Mat> imgs(flnms.size());
	vector<Point> skyVanish(flnms.size());
	vector<Point> vnspts(flnms.size());
	for(size_t i=0;i<flnms.size();++i)
	{
		imgs[i]=imread(flnms[i]+".jpg");

//		skyDetect(imgs[i]);
		skyVanish[i]=vanishFromSky(imgs[i]);
		auto lines=houghLine(imgs[i]);
		if(i>trajectoryL)
		{
			auto trjs=trajectoryDetect(imgs,(int)i);

			vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,lines,trjs);
			int x=0;int y=0;int count=0;
			for (int j = trajectoryL+1; j <=i; j++)
			{
				x+=vnspts[j].x;
				y+=vnspts[j].y;
				count++;
			}
#ifdef presentationMode_on
			Mat copy=imgs[i].clone();
			int thickness = -1;
			int lineType = 8;

			circle( copy,
			 Point(x/count,y/count),
			 4,
			 Scalar( 0, 255, 255 ),
			 thickness,
			 lineType );
			imshow("final_reslt",copy);
			imwrite(flnms[i]+"_v.jpg",copy);
	//		waitKey();
#endif
		}

		
		
	}
}

int main(int argc, char* argv[])
{
	//auto begin=GetSystemTime();
	LARGE_INTEGER m_nFreq;
	LARGE_INTEGER m_nBeginTime;
	LARGE_INTEGER nEndTime;
	long a;
	QueryPerformanceFrequency(&m_nFreq); // 获取时钟周期
	QueryPerformanceCounter(&m_nBeginTime); // 获取时钟计数

	for(auto &s:cities)
		vanishiDetectForCity(s);

	QueryPerformanceCounter(&nEndTime);
		a=(nEndTime.QuadPart-m_nBeginTime.QuadPart)*1000/m_nFreq.QuadPart;
	std::cout<<a<<endl;

//63629
	getchar();
	return 0;
}