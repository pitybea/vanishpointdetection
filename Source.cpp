#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"
#include <iostream>
#include <thread>
using namespace std;


#ifdef  step_time_Consumption
vector<long> timeStatistics(4,0.0);
LARGE_INTEGER prv,cur;
#endif
LARGE_INTEGER m_nFreq; //dirty and quick

static string cities[3]={"ny","paris","kyoto"};

inline long toReadableTime(LARGE_INTEGER m_nBeginTime,
	LARGE_INTEGER nEndTime)
{
	return (nEndTime.QuadPart-m_nBeginTime.QuadPart)*1000/m_nFreq.QuadPart;
}
#ifdef  step_time_Consumption
inline void timeSta(LARGE_INTEGER & previous,LARGE_INTEGER & current,long & tperiod)
{
	QueryPerformanceCounter(&current);
	tperiod+=toReadableTime(previous,current);
	previous=current;
}
#endif

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
#ifdef  step_time_Consumption		
		QueryPerformanceCounter(&prv);
#endif
		skyVanish[i]=vanishFromSky(imgs[i]);
#ifdef  step_time_Consumption
		timeSta(prv,cur,timeStatistics[0]);
#endif
		auto lines=houghLine(imgs[i]);
#ifdef  step_time_Consumption
		timeSta(prv,cur,timeStatistics[1]);
#endif
		if(i>trajectoryL)
		{
			auto trjs=trajectoryDetect(imgs,(int)i);
#ifdef  step_time_Consumption
			timeSta(prv,cur,timeStatistics[2]);
#endif
			vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,lines,trjs);
#ifdef  step_time_Consumption
			timeSta(prv,cur,timeStatistics[3]);
#endif
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




void vanishiDetectForCity_multiThread(string s)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");

	vector<Mat> imgs(flnms.size());
	vector<Point> skyVanish(flnms.size());
	vector<Point> vnspts(flnms.size());
	vector<vector<Vec4i> > liness(flnms.size());
	vector<vector<vector<Point2f> > > trjss(flnms.size());

	for(size_t i=0;i<flnms.size();++i)
	{
		imgs[i]=imread(flnms[i]+".jpg");
		if(i<=trajectoryL)
		{
			skyVanish[i]=vanishFromSky(imgs[i]);
			liness[i]=houghLine(imgs[i]);
		}
		else
		{
		
			skyVanish[i]=vanishFromSky(imgs[i]);
			liness[i]=houghLine(imgs[i]);
			
			trjss[i]=trajectoryDetect(imgs,(int)i);
			vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,liness[i],trjss[i]);
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
	/*
	thread test[1000];

	for(int i=0;i<1000;i++)
		test[i]=thread([](int n){cout<<n<<" ";},i);

	for(auto &t:test)
		t.join();
	getchar();*/

	LARGE_INTEGER m_nBeginTime;
	LARGE_INTEGER nEndTime;
//	long a;
	QueryPerformanceFrequency(&m_nFreq); 
	QueryPerformanceCounter(&m_nBeginTime); 

	for(auto &s:cities)
		vanishiDetectForCity_multiThread(s);

	QueryPerformanceCounter(&nEndTime);

	std::cout<<toReadableTime(m_nBeginTime,nEndTime)<<endl;

#ifdef  step_time_Consumption
	for(const auto& t:timeStatistics)
		std::cout<<t<<endl;
#endif

//63629 //single thread
	/*
	5.97826087 sky detection
	16.80383698 hough line detection
	67.86950846 motion analysis
	9.348393688 information fusion

	*/
	//57186 multi thread time consumption
	//44935 motion analysis alone
	//5539 keypoint detection
	getchar();
	return 0;
}