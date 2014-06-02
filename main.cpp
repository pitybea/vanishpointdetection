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


void saveTraj_BannoFormat(const char* &oup,const vector<string>& flnms,const vector<vector<Point2f> >& trajs,const Mat & img)
{
	FILE* fp;
	fopen_s(&fp,oup,"w");
	fprintf(fp,"%d %d %d %d\n",2* flnms.size(),trajs.size(),img.cols,img.rows);
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
}


bool nonEmpty(const Point2f pt)
{
	if((pt.x<0.01)&&(pt.y<0.01))
		return false;

	return true;
}

void saveTraj_parallel(const string& inpf,const vector<vector<Point2f> >& trajs)
{
	#pragma omp parallel for
	for (int i = 0; i < trajs[0].size()-1; i++)
	{
		string fn=inpf+to_string(i)+"_"+to_string(i+1)+".txt";

		FILE* fp;
		fopen_s(&fp,fn.c_str(),"w");
		for (int j = 0; j < trajs.size(); j++)
		{
			if (nonEmpty(trajs[j][i])&&nonEmpty(trajs[j][i+1]))
			{
				fprintf(fp,"%d %lf %lf %lf %lf\n",j,trajs[j][i].x,trajs[j][i].y,trajs[j][i+1].x,trajs[j][i+1].y);
			}
		}
		fprintf(fp,"-1");
		fclose(fp);
	}
}



vector<double> Img2Serph(double x,double y)
{
	double alpha= -x*2*pi/8000+2*pi;
	double beta=  -pi*y/4000+pi/2;

	vector<double> rslt(3);
	rslt[0]=cos(beta)*cos(alpha);
	rslt[1]=cos(beta)*sin(alpha);
	rslt[2]=sin(beta);
	return rslt;
}

vector<pair<vector<double>,vector<double> > >  inMotionFileSingle(const string& fln)
{
	FILE* fp;
	fopen_s(&fp,fln.c_str(),"r");

	int trj_ind;
	fscanf_s(fp,"%d ",&trj_ind);

	vector<pair<vector<double>,vector<double> > > rslt;

	rslt.reserve(traj::kptDet_maxCorners);
	while(trj_ind!=-1)
	{
		double d1,d2,d3,d4;
		fscanf_s(fp,"%lf %lf %lf %lf\n",&d1,&d2,&d3,&d4);
		pair<vector<double>,vector<double> > one;
		one.first=Img2Serph(d1,d2);
		one.second=Img2Serph(d3,d4);
		rslt.push_back(one);
		fscanf_s(fp,"%d ",&trj_ind);
	}
	

	fclose(fp);
	return rslt;
}


vector<double> estimateMotion(const vector<pair<vector<double>,vector<double> > >& pts)
{

	vector<double> rslt; 
	rslt.resize(3,0.0);

	vector<vector< int >> xyz_ind(3);
	for (int i = 0; i < 3; i++)
	{
		xyz_ind[i].reserve(pts.size());
	}

	for (int i = 0; i < pts.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (pts[i].first[j]<dis_lmt)
			{
				xyz_ind[j].push_back(i);
			}
		}
	}
	
	for (int i = 0; i < 3; i++)
	{
		vector<double> diss;
		diss.resize(xyz_ind[i].size());
		vector<int> indx;
		indx.resize(xyz_ind[i].size(),0);
		for (int j = 0; j < xyz_ind[i].size(); j++)
		{
			diss[j]=pts[j].second[i]-pts[j].first[i];
			indx[j]=j;
		}
		FromSmall(diss,diss.size(),indx);

		int st=( perc_lmt-1.0)* diss.size()/2*(-1);
		int ed=diss.size()+( perc_lmt-1.0)* diss.size()/2;
		for (int j = st; j < ed; j++)
		{
			rslt[i]+=diss[indx[j]];
		}
		if(ed>st)
			rslt[i]/=(ed-st);
	}

	return rslt;
};


void EstimateTransofrmationsimple(const string& fln)
{
	FILE* fp;
	fopen_s(&fp,fln.c_str(),"r");
	int frm_num;
	fscanf_s(fp,"%d",&frm_num);
	fclose(fp);

	vector<string> flns(frm_num-1);
	for (int i = 0; i < frm_num-1; i++)
	{
		flns[i]=fln+to_string(i)+"_"+to_string(i+1)+".txt";
	}
	vector<vector<double> > trans(flns.size());

	#pragma omp parallel for
	for (int i = 0; i < flns.size(); i++)
	{
		cout<<flns[i]<<" ";
		auto sth=inMotionFileSingle(flns[i]);
		trans[i]=estimateMotion(sth);
	}

	for (int i = 1; i < trans.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans[i][j]+=trans[i-1][j];
		}
	}
	fopen_s(&fp,"trans.txt","w");
	fprintf(fp,"0.0 0.0 0.0\n");
	for (int i = 0; i < trans.size(); i++)
	{

		fprintf(fp,"%lf %lf %lf\n",trans[i][0],trans[i][1],trans[i][2]);
	}
	fclose(fp);
}
int main(int argc, char* argv[])
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


}

int main876(int argc, char* argv[])
{
//	_chdir("D:\\DATA\\campodia_new\\fm");

	char* inp,*oup;
	inp="img.lst";
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