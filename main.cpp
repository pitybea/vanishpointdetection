﻿#include <Windows.h>
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
#include "simulation.h"
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

vector<double> motionEstimate(vector<vector<double> >& p1,vector<vector<double> >& p2,vector<double>& dis )
{
	


	//double para[4][4];
	vector<vector<double> > para(3,vector<double>(4,0.0));
	for (int i = 0; i < p1.size(); ++i)
	{
		double a,b,c;
		double x,y,z;
		a=p1[i][0];b=p1[i][1];c=p1[i][2];
		x=p2[i][0];y=p2[i][1];z=p2[i][2];
		double d=a*x+b*y+c*z;

		double xtt[3]={x-d*a,y-d*b,z-d*c};
		x=xtt[0];y=xtt[1];z=xtt[2];
		x*=dis[i];y*=dis[i];z*=dis[i];
		//2*r + 2*x - 4*a^2*r + 2*a^4*r - 2*a^2*x + 2*a^2*b^2*r + 2*a^2*c^2*r - 4*a*b*s - 4*a*c*t - 2*a*b*y - 2*a*c*z + 2*a*b^3*s + 2*a^3*b*s + 2*a*c^3*t + 2*a^3*c*t + 2*a*b*c^2*s + 2*a*b^2*c*t
		//2*s + 2*y - 4*b^2*s + 2*b^4*s - 2*b^2*y + 2*a^2*b^2*s + 2*b^2*c^2*s - 4*a*b*r - 4*b*c*t - 2*a*b*x - 2*b*c*z + 2*a*b^3*r + 2*a^3*b*r + 2*b*c^3*t + 2*b^3*c*t + 2*a*b*c^2*r + 2*a^2*b*c*t
		//2*t + 2*z - 4*c^2*t + 2*c^4*t - 2*c^2*z + 2*a^2*c^2*t + 2*b^2*c^2*t - 4*a*c*r - 4*b*c*s - 2*a*c*x - 2*b*c*y + 2*a*c^3*r + 2*a^3*c*r + 2*b*c^3*s + 2*b^3*c*s + 2*a*b^2*c*r + 2*a^2*b*c*s
 
		para[0][0]+= 2 - 4*a*a   + 2*a*a*b*b  + 2*a*a*c*c  + 2*a*a*a*a ;
		para[0][1]+= 0 -4*a*b + 2*a*b*b*b + 2*a*a*a*b + 2*a*b*c*c;
		para[0][2]+= 0 -4*a*c + 2*a*c*c*c + 2*a*a*a*c + 2*a*b*b*c ;
		para[0][3]+= 2*x  - 2*a*a*x  - 2*a*b*y - 2*a*c*z;

		para[1][0]+= 0 -4*a*b  + 2*a*b*b*b + 2*a*a*a*b + 2*a*b*c*c*c ;
		para[1][1]+= 2 - 4*b*b + 2*b*b*b*b + 2*a*a*b*b + 2*b*b*c*c ;
		para[1][2]+= 0 -4*b*c + 2*b*c*c*c + 2*b*b*b*c + 2*a*a*b*c;
		para[1][3]+= 2*y - 2*b*b*y  - 2*a*b*x - 2*b*c*z;

		para[2][0]+= 0 - 4*a*c + 2*a*c*c*c  + 2*a*a*a*c + 2*a*b*b*c ;
		para[2][1]+= 0 - 4*b*c + 2*b*c*c*c + 2*b*b*b*c + 2*a*a*b*c;
		para[2][2]+= 2 - 4*c*c  + 2*c*c*c*c + 2*a*a*c*c + 2*b*b*c*c;
		para[2][3]+= 2*z - 2*c*c*z  - 2*a*c*x - 2*b*c*y ;


	}

	double a,b,c,d,e,f,g,h,i,j,k,l;
	a=para[0][0];
	b=para[0][1];
	c=para[0][2];
	d=para[0][3];
	
	e=para[1][0];
	f=para[1][1];
	g=para[1][2];
	h=para[1][3];

	i=para[2][0];
	j=para[2][1];
	k=para[2][2];
	l=para[2][3];
	double mx,my,mz;

	mx=-(b*g*l - b*h*k - c*f*l + c*h*j + d*f*k - d*g*j)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	my=(a*g*l - a*h*k - c*e*l + c*h*i + d*e*k - d*g*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	mz=-(a*f*l - a*h*j - b*e*l + b*h*i + d*e*j - d*f*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
 
	vector<double> rslt(3,0.0);

	rslt[0]=mx;
	rslt[1]=my;
	rslt[2]=mz;

	for (int i = 0; i < p1.size(); i++)
	{
		double a,b,c;
		double x,y,z;
		a=p1[i][0];b=p1[i][1];c=p1[i][2];
		x=p2[i][0];y=p2[i][1];z=p2[i][2];
		double d=a*x+b*y+c*z;

		double xtt[3]={x-d*a,y-d*b,z-d*c};
		
		x=rslt[0];y=rslt[1];z=rslt[2];
		d=a*x+b*y+c*z;

		double ytt[3]={x-d*a,y-d*b,z-d*c};
		dis[i]=sqrt(ytt[0]*ytt[0]+ytt[1]*ytt[1]+ytt[2]*ytt[2]) / sqrt(xtt[0]*xtt[0]+xtt[1]*xtt[1]+xtt[2]*xtt[2]);

		
	}


	return rslt;
}



int main()
{
	//_chdir("D:\\Wang\\incrementalTracking\\x64\\Release");

	vector<vector<double> > p1,p2;

	FILE* fp;
	fp=fopen("p1.txt","r");
	int n;
	fscanf(fp,"%d\n",&n);
	p1.resize(n,vector<double>(3,0.0));

	for (int i = 0; i < n; i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&p1[i][0],&p1[i][1],&p1[i][2]);
	}

	fclose(fp);

	fp=fopen("p2.txt","r");
	fscanf(fp,"%d\n",&n);
	p2.resize(n,vector<double>(3,0.0));
	for (int i = 0; i < n; i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&p2[i][0],&p2[i][1],&p2[i][2]);
	}
	
	fclose( fp);

	vector<double> rmotion(3,0.0);
	rmotion[0]=0.342077;
	rmotion[1]= 0.498291;
	rmotion[2]= 0.378039;
	vector<double> rdis(1000,0.0);
	fp=fopen("pnt.txt","r");
	for (int i = 0; i < 1000; i++)
	{
		double tx,ty,tz;
		fscanf(fp,"%lf %lf %lf\n",&tx,&ty,&tz);
		rdis[i]=sqrt(tx*tx+ty*ty+tz*tz);
	}
	fclose(fp);

	vector<double> dis(p1.size(),300.0);


	FILE* ft1,* ft2;

	ft1=fopen("distanceError.txt","w");
	ft2=fopen("estimateError","w");
	double sum=500.0;
	while(sum>0.5)
	{
		auto mot= motionEstimate (p1,p2,dis);
		double a=mot[0]-rmotion[0];
		double b=mot[1]-rmotion[1];
		double c=mot[2]-rmotion[2];

		fprintf(ft2,"%lf\n",sqrt(a*a+b*b+c*c));
		sum=0.0;
		for (int j = 0; j < 1000; j++)
		{
			sum+=abs(dis[j]-rdis[j]);
		}
		sum/=1000;
		fprintf(ft1,"%lf\n",sum);
		
	}

	fclose(ft1);
	fclose(ft2);

	return 0;
}

int main——()
{
	int pntnum=1000;
	int step=400;
	auto& w=genSomeData(200.0,400.0,0.3,0.6,pntnum,step);

	FILE* fp=fopen("sdata.txt","w");

	fprintf(fp,"%d %d\n",step,pntnum);

	//positions,points,observisions
	FILE* ft=fopen("pos.txt","w");

	auto& p=get<0>(w);
	for (int i = 0; i < p.size(); i++)
	{
		fprintf(fp,"%lf %lf %lf ",p[i][0],p[i][1],p[i][2]);
		fprintf(ft,"%lf %lf %lf\n",p[i][0],p[i][1],p[i][2]);

	}

	fclose(ft);
	ft=fopen("pnt.txt","w");
	fprintf(fp,"\n");

	auto& q=get<1>(w);
	for (int i = 0; i < q.size(); i++)
	{
		fprintf(fp,"%lf %lf %lf ",q[i][0],q[i][1],q[i][2]);
		fprintf(ft,"%lf %lf %lf\n",q[i][0],q[i][1],q[i][2]);
	}
	fprintf(fp,"\n");

	fclose(ft);
	auto& r=get<2>(w);
	for (int i = 0; i < r.size(); i++)
	{
		auto &s=r[i];

		if(i==0)
			ft=fopen("p1.txt","w");

		if(i==1)
			ft=fopen("p2.txt","w");

		if(i==1 || i==0)
			fprintf(ft,"%d\n",s.size());

		for(auto& t:s)
		{
			fprintf(fp,"%lf %lf %lf ",t[0],t[1],t[2]);
			if(i==1 || i==0)
				fprintf(ft,"%lf %lf %lf\n",t[0],t[1],t[2]);
		
		}
		if(i==1 || i==0)
			fclose(ft);
	}

	fclose(fp);

	return 0;
};


int main886(int argc, char* argv[])
{
	//_chdir("D:\\DATA\\campodia_new\\sfm");
	_chdir("D:\\DATA\\seiken0502\\mlg\\");
	char* inp,*oup;
	inp="task.lst";
	//oup="trajs.txt";

	if(argc>1)
	{
		inp=argv[1];
		oup=argv[2];
	}
	string fln(inp);

	auto flns=fileIOclass::InVectorString(fln);

	for (int i = 0; i < flns.size(); i++)
	{
		flns[i]=flns[i]+".kpts";
	}

	EstimateTransofrmationsimple(flns);

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
	return 0;
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

int main888(int argc,char* argv[])
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