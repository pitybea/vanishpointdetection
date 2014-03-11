#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"
#include <iostream>
#include <thread>
#include "trainTest.h"
#include "hog.h"
#include "../features/FeatureCCV.h"
#include "../features/FeatureCorrelogram.h"
#include "../features/FeatureHistogram.h"
#include "../features/FeatureWavelet.h"
using namespace std;


#ifdef  step_time_Consumption
vector<long> timeStatistics(4,0.0);
LARGE_INTEGER prv,cur;
#endif


void drawCircle(Mat& img,Point pt,Scalar s=Scalar( 0, 255, 255 ),int siz=4)
{
		circle( img,
				 pt,
				 siz,
				 s,
				 2,
				 8 );
}

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
	
		bool notuseSky=false;
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
				vnspts[i]=vanishPointDecide(notuseSky,imgs[i],i,skyVanish,lines,trjs,true,vcues);
			else
				vnspts[i]=vanishPointDecide(notuseSky,imgs[i],i,skyVanish,lines,trjs);
#ifdef  step_time_Consumption
			timeSta(prv,cur,timeStatistics[3]);
#endif
			int x=0;int y=0;int count=0;
			auto max=[](int a,int b)->int{return a>b?a:b;};
			for (int j = max(trajectoryL+1,i-trajectoryL*2); j <=i; j++)
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
			 7,
			 Scalar( 0, 0, 255 ),
			 thickness,
			 lineType );
			imshow("final_reslt",copy);
			imwrite(flnms[i]+"_v.jpg",copy);
			waitKey();
#endif
		}

		
		
	}
}




void vanishiDetectForCity_multiThread(string s)//cannot produce correct results, be aware
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");

	vector<Mat> imgs(flnms.size());
	vector<Point> skyVanish(flnms.size());
	vector<Point> vnspts(flnms.size());
	vector<vector<Vec4i> > liness(flnms.size());
	vector<vector<vector<Point2f> > > trjss(flnms.size());
	bool notusesky=false;
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
			vnspts[i]=vanishPointDecide(notusesky,imgs[i],i,skyVanish,liness[i],trjss[i]);
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

vector<double> imgToHog(string imgn)
{

	Mat img=imread(imgn+".jpg"),gray;
	cvtColor(img,gray,CV_BGR2GRAY);
	vector<vector<int> > grayim(gray.rows,vector<int>(gray.cols));
	for (int i = 0; i < grayim.size(); i++)
	{
		for (int j = 0; j < grayim[0].size(); j++)
		{
			grayim[i][j]=static_cast<int>(gray.at<uchar>(Point(j,i)));
		}
	}	
	return hogFeatureSpecial(grayim);
}
vector<double> imgToVector(string imgn)
{
	vector<double> rslt(50*50);
	Mat img=imread(imgn+".jpg"),gray;
	cvtColor(img,gray,CV_BGR2GRAY);
	resize(gray,gray,Size(50,50));
	for (int i = 0; i < 50; i++)
	{
		for (int j = 0; j < 50; j++)
		{
			rslt[i*50+j]=(double)static_cast<int>(gray.at<uchar>(Point(i,j)))/255.0;
		}
	}

	return rslt;
}



void turnDataFormat_forCities()
{
	vector<vector<double> > traindata;
	vector<int> trainlbs;
	vector<string> trainnms;
	vector<vector<double> > testdata;
	vector<int> testlbs;
	vector<string> testnms;

	int index=0;
	for (const auto& s:cities)
	{
		_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());
		vector<string> trains=fileIOclass::InVectorString("train.txt");
		vector<string> tests=fileIOclass::InVectorString("test.txt");
		for(const auto &sth: trains)
		{
			printf("%s ",sth.c_str());
			traindata.push_back(imgToVector(sth));
			trainlbs.push_back(index);
			trainnms.push_back(s+"/"+sth);
		}
		for(const auto &sth:tests)
		{
			
			printf("%s ",sth.c_str());
			testdata.push_back(imgToVector(sth));
			testlbs.push_back(index);
			testnms.push_back(s+"/"+sth);
		}
		++index;
	}
	_chdir("E:\\vanish\\dataset\\");
	fileIOclass::OutVectorInt("alltrainlbs",trainlbs);
	fileIOclass::OutVectorString("alltrainnames",trainnms);
	fileIOclass::OutVectorSDouble("alltraindata",traindata);
	
	fileIOclass::OutVectorInt("alltestlbs",testlbs);
	fileIOclass::OutVectorString("alltestnames",testnms);
	fileIOclass::OutVectorSDouble("alltestdata",testdata);

}



void turnHog_forCities()
{
	vector<vector<double> > traindata;
	vector<int> trainlbs;
//	vector<string> trainnms;
	vector<vector<double> > testdata;
	vector<int> testlbs;
	//vector<string> testnms;

	int index=0;
	for (const auto& s:cities)
	{
		_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());
		vector<string> trains=fileIOclass::InVectorString("train.txt");
		vector<string> tests=fileIOclass::InVectorString("test.txt");
		for(const auto &sth: trains)
		{
			//printf("%s ",sth.c_str());
			traindata.push_back(imgToHog (sth));
			trainlbs.push_back(index);
		//	trainnms.push_back(s+"/"+sth);
		}
		for(const auto &sth:tests)
		{
			
			//printf("%s ",sth.c_str());
			testdata.push_back(imgToHog(sth));
			testlbs.push_back(index);
			//testnms.push_back(s+"/"+sth);
		}
		++index;
	}
	_chdir("E:\\vanish\\dataset\\");
	auto prog=[](string s,vector<vector<double> > data,vector<int> lbs)
	{
		FILE* fp;
		fopen_s(&fp,s.c_str(),"w");
		for (int i = 0; i < data.size(); i++)
		{
			fprintf_s(fp,"%d ",lbs[i]);
			for (int j = 0; j < data[i].size(); j++)
			{
				fprintf_s(fp,"%d:%lf ",(j+1),data[i][j]);
			}
			fprintf(fp,"\n");
		}
		fclose(fp);
	};
	prog("forLTrain",traindata,trainlbs);
	prog("forLTest",testdata,testlbs);
	//fileIOclass::OutVectorInt("alltrainlbs",trainlbs);
	//fileIOclass::OutVectorString("alltrainnames",trainnms);
	//fileIOclass::OutVectorSDouble("alltrainhog",traindata);
	
	//fileIOclass::OutVectorInt("alltestlbs",testlbs);
	//fileIOclass::OutVectorString("alltestnames",testnms);
	//fileIOclass::OutVectorSDouble("alltesthog",testdata);

}
bool imageIntoRGB(const Mat& img,vector<vector<vector<int> > >& pixels)
{
	auto c=img.channels();
	for (int i = 0; i < img.rows; i++) //height
	{
		for (int j = 0; j < img.cols; j++) //width
		{
			Vec3b v3=img.at<Vec3b>(i,j); //BGR at<width,height>
			for (int k = 0; k < c; k++)
			{
				pixels[j][i][k]=v3[k];
			}
			
		}
	}
	return true;
}

void turnFeature_forCities(string feaNm, vector<double> (*feaFunc)(const vector< vector<vector<int> > >& ))
{
	vector<vector<double> > traindata;
	vector<int> trainlbs;
//	vector<string> trainnms;
	vector<vector<double> > testdata;
	vector<int> testlbs;
	//vector<string> testnms;

	int index=0;
	
			
		//	return feaFunc(pixels);
	
	for (const auto& s:cities)
	{
		_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());
		vector<string> trains=fileIOclass::InVectorString("train.txt");
		vector<string> tests=fileIOclass::InVectorString("test.txt");
		
		for(const auto &sth: trains)
		{
			printf("%s ",sth.c_str());
			
			//	return fea(pixels);
			Mat img=imread(sth+".jpg");
			vector<vector<vector<int> > > pixels(vector<vector<vector<int> > >(img.cols,vector<vector<int> >(img.rows,vector<int>(img.channels(),0))));
			imageIntoRGB(img,pixels);
			traindata.push_back(feaFunc(pixels));
			trainlbs.push_back(index);
		//	trainnms.push_back(s+"/"+sth);
		}
		for(const auto &sth:tests)
		{
			
			printf("%s ",sth.c_str());
				Mat img=imread(sth+".jpg");
			vector<vector<vector<int> > > pixels(vector<vector<vector<int> > >(img.cols,vector<vector<int> >(img.rows,vector<int>(img.channels(),0))));
			imageIntoRGB(img,pixels);
			testdata.push_back(feaFunc(pixels));
			testlbs.push_back(index);
			//testnms.push_back(s+"/"+sth);
		}
		++index;
		printf("%d ",index);
	}
	_chdir("E:\\vanish\\dataset\\");
	auto prog=[](string s,vector<vector<double> > data,vector<int> lbs)
	{
		FILE* fp;
		fopen_s(&fp,s.c_str(),"w");
		for (int i = 0; i < data.size(); i++)
		{
			fprintf_s(fp,"%d ",lbs[i]);
			for (int j = 0; j < data[i].size(); j++)
			{
				fprintf_s(fp,"%d:%lf ",(j+1),data[i][j]);
			}
			fprintf(fp,"\n");
		}
		fclose(fp);
	};
	prog(feaNm+"_forLTrain",traindata,trainlbs);
	prog(feaNm+"forLTest",testdata,testlbs);
	
}



void statisticAbout_classification(string fnm)
{
	_chdir("E:\\vanish\\dataset");
	vector<int> allrslt=fileIOclass::InVectorInt(fnm+".rslt");
	vector<vector<int> > stt(300,vector<int>(3,0));
	for (int i = 0; i < 300; i++) //dirty and quick
	{
		for (int j = 0; j < 100; j++) //dirty and quick
		{
			++stt[i][allrslt[i*100+j]];
		}
	}
	vector<int> output(300);
	for (int i = 0; i < 300; i++)
	{
		int tem=0;
		for (int j = 0; j < 3; j++)
		{
			if(stt[i][j]>tem)
			{
				tem=stt[i][j];
				output[i]=j;
			}
		}
	}
	fileIOclass::OutVectorInt(fnm+"_all.rslt",output);
}

void main______()
{
	const string feaType[4]={"ccv","cor","his","wav"};
	for(auto& sth:feaType)
	statisticAbout_classification(sth);
}
void main____()
{
//	statisticAbout_classification();
		const string feaType[4]={"ccv","cor","his","wav"};
		typedef vector<double> (*feaFunc)(const vector< vector<vector<int> > >& );

		const feaFunc feafuncs[4]={CFeatureCCV::GenerateFeature,CFeatureCorrelogram::GenerateFeature,CFeatureHistogram::GenerateFeature,CFeatureWavelet::GenerateFeature};
	
				
	
	//	for (int i = 0; i < 4; i++)
		{
	//		turnFeature_forCities(feaType[0],feafuncs[0]);
	//		turnFeature_forCities(feaType[1],feafuncs[1]);
	//		turnFeature_forCities(feaType[2],feafuncs[2]);
			turnFeature_forCities(feaType[3],feafuncs[3]);
		}

	//turnDataFormat_forCities();
//	turnHog_forCities();

	//_chdir("E:\\vanish\\dataset\\");
}

void trainTest_forCity(string s,double edgeratio=traintest::edge_ratio)
{
	_chdir(((string)"E:\\vanish\\dataset\\"+s).c_str());

	auto flnms=fileIOclass::InVectorString("img.lst");
	vector<Point> trainPoss;
	vector<Point> testPoss;
	vector<Mat> trainImgs;
	vector<Mat> testImgs;
	vector<string> trainnms;
	vector<string> testnms;
	int tm1=0,tm2=0;
	for(const auto &imgn:flnms)
	{
		printf("%s ",imgn.c_str());
		Mat img=imread(imgn+".jpg");
		SampleExamples(img,trainPoss,testPoss,trainImgs,testImgs,edgeratio);
		for (int j = tm1; j < trainPoss.size(); j++)
		{
			drawCircle(img,trainPoss[j],Scalar(0,255,0),5);
			trainnms.push_back(imgn+"_train_"+to_string(j-tm1));
		}
		for (int j = tm2; j < testPoss.size(); j++)
		{
			drawCircle(img,testPoss[j],Scalar(0,0,255),5);
			testnms.push_back(imgn+"_test_"+to_string(j-tm1));
		}
		

		tm1=trainPoss.size();
		tm2=trainPoss.size();

		imshow("show",img);
		waitKey();
	}
	auto func=[](string n,const vector<string>& nms,const vector<Mat>& imgs)
	{
		FILE* fp;
		fopen_s(&fp,(n+".txt").c_str(),"w");
		fprintf(fp,"%d\n",nms.size());
		for(int i=0;i<nms.size();i++)
		{
			fprintf(fp,"%s\n",nms[i].c_str());
			imwrite(nms[i]+".jpg",imgs[i]);
		}

		fclose(fp);
	};
	func("train",trainnms,trainImgs);
	func("test",testnms,testImgs);
}


int main_(int argc, char* argv[])
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

//	vanishiDetectForCity(cities[0]);
//	evluate_forCity(cities[1]);

	for(auto &s:cities)
	{	
//		vanishiDetectForCity(s);
		//markGroundTruesForCities(s);
	
	//	vanishiDetectForCity(s);
	//	evluate_forCity(s);
	//	trainTest_forCity(cities[0]);
	//	trainTest_forCity(cities[1],0.09);
		trainTest_forCity(cities[2],0.085);
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