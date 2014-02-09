#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"
#include <iostream>
using namespace std;

static string cities[3]={"ny","paris","kyoto"};

vector<long> timeStatistics(4,0.0);

LARGE_INTEGER m_nFreq; //dirty and quick

LARGE_INTEGER prv,cur;

inline long toReadableTime(LARGE_INTEGER m_nBeginTime,
	LARGE_INTEGER nEndTime)
{
	return (nEndTime.QuadPart-m_nBeginTime.QuadPart)*1000/m_nFreq.QuadPart;
}
inline void timeSta(LARGE_INTEGER & previous,LARGE_INTEGER & current,long & tperiod)
{
	QueryPerformanceCounter(&current);
	tperiod+=toReadableTime(previous,current);
	previous=current;
}
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
		
		QueryPerformanceCounter(&prv);
		skyVanish[i]=vanishFromSky(imgs[i]);
		timeSta(prv,cur,timeStatistics[0]);
		auto lines=houghLine(imgs[i]);
		timeSta(prv,cur,timeStatistics[1]);
		if(i>trajectoryL)
		{
			auto trjs=trajectoryDetect(imgs,(int)i);
			timeSta(prv,cur,timeStatistics[2]);

			vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,lines,trjs);
			timeSta(prv,cur,timeStatistics[3]);

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

	LARGE_INTEGER m_nBeginTime;
	LARGE_INTEGER nEndTime;
//	long a;
	QueryPerformanceFrequency(&m_nFreq); 
	QueryPerformanceCounter(&m_nBeginTime); 

	for(auto &s:cities)
		vanishiDetectForCity(s);

	QueryPerformanceCounter(&nEndTime);

	std::cout<<toReadableTime(m_nBeginTime,nEndTime)<<endl;
	for(const auto& t:timeStatistics)
		std::cout<<t<<endl;

//63629
	/*
	5.97826087
	16.80383698
	67.86950846
	9.348393688

	*/
	getchar();
	return 0;
}