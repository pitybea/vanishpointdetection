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
#include <math.h>
#include <map>
#include "sfm.inl"
using namespace sfm;

using namespace traj;
using namespace cv;


#include "parallelSetting.inl"
using namespace parallel;


#include "motionEstimate.h"

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
		imwrite("traj.jpg",copy);
		//waitKey();
}

template<class T>
vector<vector<T> > splitVectorIntoMultiWith_1_Overlap(const vector<T> & inp, int n)
{

	assert(n>=2);
	vector<vector<T> > rslt;

	auto pos=inp.begin();

	if(n>=inp.size())
	{
		rslt.push_back(vector<T>(inp));
	}
	else
	while (pos<inp.end())
	{
		if( pos >= (inp.end()-n) )
		{
			rslt.push_back(vector<T> (pos,inp.end() ) );
			pos=inp.end();
		}
		else
		{
			rslt.push_back(vector<T> (pos,pos+n) );
			pos+=(n-1);
		}
	}

	return rslt;
}

vector<string> readFileByLine(char* inpf)
{

	vector<string> rslt;

	FILE* fp=fopen(inpf,"r");
	
	if(fp!=NULL)
	{

		char tem[100];
	
		while(fscanf(fp,"%s\n",&tem)!=EOF)
		{
			string s(tem);
			rslt.push_back(s);
		
		}
		fclose(fp);
	}

	

	return rslt;
}


vector<string> compareAndCalRest(const vector<string>& vs1,const vector<string>& vs2,int& p1)
{
	vector<string> rslt;

	p1=0;

	for (int i = 0; i < vs2.size(); i++)
	{
		if( vs2[i].compare(vs1[p1])==0 )
			++p1;
	}

	rslt=vector<string>(vs1.begin()+p1,vs1.end());

	return rslt;
}





int main712(int argc, char* argv[])
{
	//_chdir("D:\\DATA\\campodia_new\\sfm");

	char* inp,*oup;
	inp="allimg.lst";
	oup="trajs.txt";

	if(argc>1)
	{
		inp=argv[1];
		oup=argv[2];
	}
	string fln(inp);
	EstimateTransofrmationsimple(fln);

	return 0;
}




double dis(const vector<double>& a,const vector<double>& b)
{
		assert(a.size()==b.size());
		double sumn(0.0);
		for (int i = 0; i < a.size(); i++)
		{
			double n=a[i]-b[i];
			sumn+=n*n;
		}
		return std::sqrt((double)sumn);
}

vector<double> histByDiss(vector<double> diss)
{
	vector<double>  rslt(200,0.0);

	double step=1.0/diss.size();

	for (int i = 0; i < diss.size(); i++)
	{
		if(diss[i]<0.3047)
			rslt[diss[i]/0.0035]+=step;
		else
		{
			int pp=(diss[i]-0.3047)/0.1523;
			if(pp>199)
				pp=199;
			rslt[pp]+=step;
		}
	}


	return rslt;

}


int main117()
{
	#ifdef _DEBUG
	_chdir("D:\\DATA\\seiken0502\\sf\\");
#endif

	auto func=[](string s1,string s2,string s3)
	{
		auto v1=fileIOclass::InVectorSDouble(s1);
		auto v2=fileIOclass::InVectorSDouble(s2);

		
		FILE* fp=fopen(s3.c_str(),"w");
		auto princ=[&](vector<double> v)
		{
			for(int i=0;i<v.size();i++)
			{
				fprintf(fp,"%d:%lf ",i+1,v[i]);
			}
		};
		for (int i = 0; i < v1.size(); i++)
		{
			fprintf(fp,"+1 ");
			princ(v1[i]);
			fprintf(fp,"\n");
		}
		for (int i = 0; i < v2.size(); i++)
		{
			fprintf(fp,"-1 ");
			princ(v2[i]);
			fprintf(fp,"\n");
		}

		fclose(fp);
	};
	func("positive_train.feature","negative_train.feature","train_feature");
	func("positive_test.feature","negative_test.feature","test_feature");
}

int main711()
{
#ifdef _DEBUG
	_chdir("D:\\DATA\\seiken0502\\sf\\");
#endif

	//auto ptnlst=fileIOclass::InVectorString("positive_train.lst");
	auto func=[&](string inp,string oup)
	{
		auto fnlst=fileIOclass::InVectorString(inp);
		vector<vector<double> > features(fnlst.size());



		vector<double> diss;
		diss.reserve(4000);

		#pragma omp parallel for
		for(int i=0;i<fnlst.size();i++)
		{
			auto sd=inMotionFileSingle( fnlst[i]+".tsk.kpts");
			diss.resize(sd.size());
			for (int j = 0; j < sd.size(); j++)
			{
				diss[j]=dis(sd[j].first,sd[j].second);

			}
			features[i]=histByDiss(diss);
			if(i%100==0)
				cout<<i<<" ";
		}
		fileIOclass::OutVectorSDouble(oup,features);

	};
	
//	func("positive_train.lst","positive_train.feature");
	//func("positive_test.lst","positive_test.feature");
	//func("negative_train.lst","negative_train.feature");
	func("negative_test.lst","negative_test.feature");

	//getchar();
	return 0;
}

int main142(int argc,char*argv[])
{
#ifdef _DEBUG
	_chdir("D:\\DATA\\seiken0502\\sf\\");
#endif

	auto allst=fileIOclass::InVectorString("allimg.lst");
	auto sel=fileIOclass::InVectorString("selFrm.lst");

	map<string,int> pos;
	for(auto& s:sel)
		pos[s]=1;

	vector<string> psamples;
	vector<string> nsamples;
	for (int i = 0; i < 10000; i++)
	{
		if(pos.count(allst[i]))
			psamples.push_back(allst[i]);
		else
			nsamples.push_back(allst[i]);
	}
	fileIOclass::OutVectorString("positive_train.lst",psamples);
	fileIOclass::OutVectorString("negative_train.lst",nsamples);

	psamples.clear();
	nsamples.clear();

	for (int i = 10000; i < allst.size()-1; i++)
	{
		if(pos.count(allst[i]))
			psamples.push_back(allst[i]);
		else
			nsamples.push_back(allst[i]);
	}
	fileIOclass::OutVectorString("positive_test.lst",psamples);
	fileIOclass::OutVectorString("negative_test.lst",nsamples);

	return 0;

}

int main(int argc,char* argv[])
{
	char* inp;
#ifdef _DEBUG
	_chdir("D:\\DATA\\seiken0502\\sf\\");

	
#endif 
	inp="ladybug_panoramic_000000.jpg.tsk";
	if(argc>1)
		inp=argv[1];
	
	auto fls=fileIOclass::InVectorString(inp);

	assert(fls.size()==2);

	string s(inp);
	simpleTrackPrint(fls,s);

	cout<<s<<endl;
	return 0;
}

int main_fort(int argc, char* argv[])
{
//	_chdir("D:\\DATA\\campodia_new\\fm");

#ifdef _DEBUG
	_chdir("E:\\results\\ssf");
#endif 
	char* inp,*log,*max_index;
	inp="allimg.lst";
	log="log.txt";
	max_index="maxIndx.txt";

	int local_patchSiz=patch_size;

	if(argc>1)
	{
		inp=argv[1];
		log=argv[2];
		max_index=argv[3];
	}

	if(argc>4)
	{
		int	temI=atoi(argv[4]);
		if (temI<patch_size)
			local_patchSiz=temI;
	}

	auto flnms=fileIOclass::InVectorString(inp);

	int start_index;
	auto temStrs=compareAndCalRest( flnms,readFileByLine(log),start_index);

	vector<int> temInts;
	temInts.reserve(temStrs.size());
	for (int i = 0; i < temStrs.size(); i++)
	{
		temInts.push_back(start_index+i);
	}

	auto patchNms=splitVectorIntoMultiWith_1_Overlap( temStrs,local_patchSiz);
	auto patchIndx=splitVectorIntoMultiWith_1_Overlap(temInts,local_patchSiz);


	patchDealSave(patchNms,patchIndx,inp,log,max_index);

	return 0;
	
}


int main623(int argc, char* argv[])
{
//	_chdir("D:\\DATA\\campodia_new\\fm");

#ifdef _DEBUG
	_chdir("E:\\results\\ssf");
#endif 





	char* inp,*oup;
	inp="allimg.lst";
	oup="trajs.txt";

	if(argc>1)
	{
		inp=argv[1];
		oup=argv[2];
	}

	auto flnms=fileIOclass::InVectorString(inp);
//	vector<Mat> imgs(flnms.size());

	//flnms.erase(flnms.begin()+4,flnms.end());
	
	auto trajs=incrementalTrajectoryDetect(flnms);

	vector<bool> lbs(trajs.size(),false);
	for (int i = 0; i < trajs.size(); i++)
	{
		if(trajs[i][0].x>0.1)
			lbs[i]=true;
	}

	

	removeStaticTrajectories(trajs);

	string fns(inp);
	saveTraj_parallel(fns,trajs);

	/*
	vector<vector<Point2f> > effect_trajs(trajs.size());

#pragma omp parallel for
	for (int i = 0; i < trajs.size(); i++)
	{
		cout<<i<<"\t";
		effect_trajs[i]= effectTraj (trajs[i]);
	}

	Mat img=imread(flnms[0]);
	drawTrajs_ok(img,effect_trajs);
	imshow("whatever",img);
	imwrite("results.jpg",img);

	saveTraj_BannoFormat(flnms,trajs,img);
	*/
//	waitKey();
	//testsimple(imgs[0],imgs[1]);
	
	return 1;
}