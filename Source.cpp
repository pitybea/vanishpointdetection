#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"
#include <iostream>
#include <thread>
#include "trainTest.h"
using namespace std;


#ifdef  step_time_Consumption
vector<long> timeStatistics(4,0.0);
LARGE_INTEGER prv,cur;
#endif

LARGE_INTEGER m_nFreq; //dirty and quick

static string cities[3]={"ny","paris","kyoto"};

bool statistics_forCues=true;

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

void markGroundTruesForCities(string ss)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+ss).c_str());
	auto flnms=fileIOclass::InVectorString("img.lst");
	for(const auto&s:flnms)
	{
		Mat img=imread(s+".jpg");
		namedWindow("result");
		imshow("result",img);

		

		Point p;
//		func(3,4);
		setMouseCallback("result",[](int e,int x,int y,int d,void* ptr){
			if  (e == EVENT_LBUTTONDOWN )	
			{
				auto& p=((pair<Point&,Mat&>*)ptr)->first;
				p.x=x,p.y=y;
				int thickness = -1;
				int lineType = 8;
				auto& img=((pair<Point&,Mat&>*)ptr)->second;
				circle( img,
				 p,
				 4,
				 Scalar( 0, 255, 255 ),
				 thickness,
				 lineType );
				imshow("result",img);
			}//			(*(void (*)(int,int))ptr)(x,y);	
			
			
		},&pair<Point&,Mat&>(p,img));
		
		waitKey();
		FILE* fp;
		fopen_s(&fp,(s+"_gth.txt").c_str(),"w");
		fprintf(fp,"%d %d",p.x,p.y);
		fclose(fp);
	}
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
		printf("%s ",flnms[i].c_str());
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
			vector<Point> vcues;
			if(statistics_forCues)
				vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,lines,trjs,true,vcues);
			else
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
			auto rslt =	Point(x/count,y/count);
			if(statistics_forCues)
			{
				FILE* fp;
				fopen_s(&fp,(flnms[i]+"_vpts.txt").c_str(),"w");
				fprintf(fp,"%d %d\n",rslt.x,rslt.y);
				for (size_t j = 0; j < vcues.size(); j++)
				{
					fprintf(fp,"%d %d\n",vcues[j].x,vcues[j].y);
				}
				fclose (fp);

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
		printf("%s ",flnms[i].c_str());
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
			auto rslt =	Point(x/count,y/count);
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

void evluate_forCity(string s)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");
	vector<vector<double>> rslts(4,vector<double>(flnms.size()));
	for(size_t i=0;i<flnms.size();i++)
	{
		string& ss=flnms[i];
		FILE* fp1,*fp2;
		Point gt;
		fopen_s(&fp1,(ss+"_gth.txt").c_str(),"r");
		
		fopen_s(&fp2,(ss+"_vpts.txt").c_str(),"r");
		if(fp1&&fp2)
		{
			fscanf_s(fp1,"%d %d",&gt.x,&gt.y);
			vector<Point> pts(4);
			for (int j = 0; j < 4; j++)
			{
				fscanf_s(fp2,"%d %d\n",&pts[j].x,&pts[j].y);
				rslts[j][i]=cv::norm(pts[j]-gt);
			}
			
			fclose(fp1);fclose(fp2);
		}
	}
	FILE* fp;
	fopen_s(&fp,(s+"_rslts.txt").c_str(),"w");
	for (int i = 0; i < flnms.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			fprintf(fp,"%lf ",rslts[j][i]);
		}
		fprintf(fp,"\n");
	}
	fclose(fp);
}

void trainTest_forCity(string s)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");
	vector<Point> trainPoss;
	vector<Point> testPoss;
	vector<Mat> trainImgs;
	vector<Mat> testImgs;
	for(const auto &imgn:flnms)
	{
		Mat img=imread(imgn+".jpg");
		SampleExamples(img,
		imshow("show",img);
		waitKey();
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
		//markGroundTruesForCities(s);
	{
	//	vanishiDetectForCity(s);
	//		evluate_forCity(s);
	}
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