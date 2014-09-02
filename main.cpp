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

#include "simulation.h"
#include "sfm.inl"
using namespace sfm;

using namespace traj;
using namespace cv;

#include <algorithm>
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
		//2*r + 2*x - 4*a*a*r + 2*a*a*a*a*r - 2*a*a*x + 2*a*a*b*b*r + 2*a*a*c*c*r - 4*a*b*s - 4*a*c*t - 2*a*b*y - 2*a*c*z + 2*a*b*b*b*s + 2*a*a*a*b*s + 2*a*c*c*c*t + 2*a*a*a*c*t + 2*a*b*c*c*s + 2*a*b*b*c*t
		//2*s + 2*y - 4*b*b*s + 2*b*b*b*b*s - 2*b*b*y + 2*a*a*b*b*s + 2*b*b*c*c*s - 4*a*b*r - 4*b*c*t - 2*a*b*x - 2*b*c*z + 2*a*b*b*b*r + 2*a*a*a*b*r + 2*b*c*c*c*t + 2*b*b*b*c*t + 2*a*b*c*c*r + 2*a*a*b*c*t
		//2*t + 2*z - 4*c*c*t + 2*c*c*c*c*t - 2*c*c*z + 2*a*a*c*c*t + 2*b*b*c*c*t - 4*a*c*r - 4*b*c*s - 2*a*c*x - 2*b*c*y + 2*a*c*c*c*r + 2*a*a*a*c*r + 2*b*c*c*c*s + 2*b*b*b*c*s + 2*a*b*b*c*r + 2*a*a*b*c*s
 
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

double mpow(double a,int b)
{
	if(b==0)
		return 1.0;

	if(b==1)
		return a;

	if(b%2==0)
	{
		double t=mpow(a,b/2);
		return t*t;
	}
	else
	{
		double t=mpow(a,b/2);
		return t*t*a;
	}
}

vector<double> motionEstimate3(vector<vector<double> >& p1,vector<vector<double> >& p2,vector<double>& dis )
{
	vector<vector<double> > para(3,vector<double>(4,0.0));
	for (int i = 0; i < p1.size(); i++)
	{
		double x,y,z;
		double a,b,c;


		a=p1[i][0];b=p1[i][1];c=p1[i][2];
		x=p2[i][0];y=p2[i][1];z=p2[i][2];

		double d=a*x+b*y+c*z;

		a=x-d*a;b=y-d*b;c=z-d*c;

		d=a*a+b*b+c*c;



		double L=dis[i];

	//	pow(2.0,2);
		para[0][0]+=1.0/(L*L)*(a*a)*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[0][1]+=1.0/(L*L)*a*b*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[0][2]+=1.0/(L*L)*a*c*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[0][3]+=(a*(a*a+b*b+c*c)*2.0)/(L*d);

		para[1][0]+=1.0/(L*L)*a*b*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[1][1]+=1.0/(L*L)*(b*b)*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[1][2]+=1.0/(L*L)*b*c*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[1][3]+=(b*(a*a+b*b+c*c)*2.0)/(L*d);

		para[2][0]+=1.0/(L*L)*a*c*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[2][1]+=1.0/(L*L)*b*c*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[2][2]+=1.0/(L*L)*(c*c)*1.0/(d*d)*(a*a+b*b+c*c)*2.0;
		para[2][3]=(c*(a*a+b*b+c*c)*2.0)/(L*d);
		

		//cout<<a*a+b*b+c*c<<" "<<x*x+y*y+z*z<<endl;

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
		double x,y,z;
		double a,b,c;


		a=p1[i][0];b=p1[i][1];c=p1[i][2];
		x=p2[i][0];y=p2[i][1];z=p2[i][2];

		double d=a*x+b*y+c*z;

		a=x-d*a;b=y-d*b;c=z-d*c;



		x=rslt[0];y=rslt[1];z=rslt[2];
		d=a*x+b*y+c*z/(a*a+b*b+c*c);

		double ytt[3]={d*a,d*b,d*c};
		dis[i]=sqrt(ytt[0]*ytt[0]+ytt[1]*ytt[1]+ytt[2]*ytt[2]) / sqrt(a*a+b*b+c*c);

		
	}


	return rslt;

}

vector<double> motionEstimate2(vector<vector<double> >& p1,vector<vector<double> >& p2,vector<double>& dis )
{


	vector<vector<double> > para(3,vector<double>(4,0.0));
	for (int i = 0; i < p1.size(); i++)
	{
		double x,y,z,a,b,c;

		a=p1[i][0];b=p1[i][1];c=p1[i][2];
		x=p2[i][0];y=p2[i][1];z=p2[i][2];
		double d=a*x+b*y+c*z;

		double xt[3]={x-d*a,y-d*b,z-d*c};

		double L=dis[i];
		x=xt[0]*L;y=xt[1]*L;z=xt[2]*L;

		

		para[0][0]+=  (2.0  - (4.0*a*a)  + (2.0*a*a*a*a)  + (2.0*a*a*b*b)  + (2.0*a*a*c*c));
		para[0][1]+=( (2.0*a*b*b*b)  + (2.0*a*a*a*b)  - (4.0*a*b)  + (2.0*a*b*c*c)  ) ;
		para[0][2]+=((2.0*a*c*c*c)  + (2.0*a*a*a*c)  - (4.0*a*c)  + (2.0*a*b*b*c)  );
		para[0][3]+=((2.0*x)  - (2.0*a*a*x)  - (2.0*a*b*y)  - (2.0*a*c*z)  );

		para[1][0]+=((2.0*a*b*b*b)  + (2.0*a*a*a*b)  - (4.0*a*b)  + (2.0*a*b*c*c) );
		para[1][1]+=(2.0  - (4.0*b*b)  + (2.0*b*b*b*b)  + (2.0*a*a*b*b)  + (2.0*b*b*c*c) );
		para[1][2]+=((2.0*b*c*c*c)  + (2.0*b*b*b*c)  - (4.0*b*c)  + (2.0*a*a*b*c) );
		para[1][3]+=((2.0*y)  - (2.0*b*b*y)  - (2.0*a*b*x)  - (2.0*b*c*z) );

		para[2][0]+=((2.0*a*c*c*c)  + (2.0*a*a*a*c)  - (4.0*a*c)  + (2.0*a*b*b*c) );
		para[2][1]+=((2.0*b*c*c*c)  + (2.0*b*b*b*c)  - (4.0*b*c)  + (2.0*a*a*b*c) );
		para[2][2]+=(2.0  - (4.0*c*c)  + (2.0*c*c*c*c)  + (2.0*a*a*c*c)  + (2.0*b*b*c*c) );
		para[2][3]+=((2.0*z)  - (2.0*c*c*z)  - (2.0*a*c*x)  - (2.0*b*c*y) );


	}

	double a11,a12,a13,a14,a21,a22,a23,a24,a31,a32,a33,a34;
	a11=para[0][0];
	a12=para[0][1];
	a13=para[0][2];
	a14=para[0][3];

	a21=para[1][0];
	a22=para[1][1];
	a23=para[1][2];
	a24=para[1][3];

	a31=para[2][0];
	a32=para[2][1];
	a33=para[2][2];
	a34=para[2][3];

	double w1,w2,w3;
	w1 = -(a12*a23*a34 - a12*a24*a33 - a13*a22*a34 + a13*a24*a32 + a14*a22*a33 - a14*a23*a32)/(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);
 
	w2=(a11*a23*a34 - a11*a24*a33 - a13*a21*a34 + a13*a24*a31 + a14*a21*a33 - a14*a23*a31)/(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);

	w3=-(a11*a22*a34 - a11*a24*a32 - a12*a21*a34 + a12*a24*a31 + a14*a21*a32 - a14*a22*a31)/(a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);
 
	vector<double> rslt(3,0.0);

	rslt[0]=w1;
	rslt[1]=w2;
	rslt[2]=w3;

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


int maindis()
{
	_chdir("D:\\Wang\\incrementalTracking\\x64\\Release");

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
	ft2=fopen("estimateError.txt","w");
	double sum=500.0;
	for (int i = 0; i < 1000; i++)
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


int main()
{
	_chdir("F:\\tss");
	auto tasks=fileIOclass::InVectorString("task.lst");
	auto feaNames=fileIOclass::InVectorString("orb.lst");
	auto mathces=fileIOclass::InVectorString("mtchc.lst");

	vector<vector<vector<int> > > correpss(mathces.size());

	for (int i = 0; i < correpss.size(); i++)
	{
		correpss[i]=fileIOclass::InVectorSInt(mathces[i],2);
	}
	
	vector<vector<vector<int> > > feas(feaNames.size());

	for (int i = 0; i < feaNames.size(); i++)
	{
		feas[i]=fileIOclass::InVectorSInt(feaNames[i],2);
	}

	vector<vector<Point> > features(feas.size());
	vector<map<int,int> > sth(correpss.size());

	for (int i = 0; i < feas.size(); i++)
	{
		features[i].resize(feas[i].size());
		for (int j = 0; j < feas[i].size(); j++)
		{

			features[i][j]=Point(feas[i][j][0],feas[i][j][1]);
		}
	}
	for(int i=0;i<correpss.size();++i)
	{
		for(int j=0;j<correpss[i].size();++j)
			sth[i][correpss[i][j][0]]=correpss[i][j][1];
	}
	auto trajs=incrementalTrajectoryDetect(features,sth);

	auto& ind=trajs.first;
	auto& track=trajs.second;

	vector<FILE*> ft(feaNames.size());

	for(int i=0;i<ft.size();i++)
		ft[i]=fopen((feaNames[i]+".gpts").c_str(),"w");

	for(int i=0;i<track.size();++i)
	{
		for (int j = 0; j < track[i].size(); j++)
		{
			fprintf(ft[j+ind[i].first],"%d %d %d\n",i,track[i][j].x,track[i][j].y);
		}
	}
	
	for(int i=0;i<ft.size();i++)
		fclose(ft[i]);
	FILE* fp=fopen("goldenpnt.lst","w");
	fprintf(fp,"%d\n",ind.size());
	for(int i=0;i<ind.size();i++)
	{
		for(int j=ind[i].first;j<=ind[i].second;++j)
			fprintf(fp,"%d ",j);

		fprintf(fp,"\n");
	}	


	fclose(fp);


	

	return 0;
}

/*
int maidfn()
{
	_chdir("F:\\tss");

	auto imgNames=fileIOclass::InVectorString("allimg.lst");
	auto feaNames=fileIOclass::InVectorString("orb.lst");
	vector<string> corrName(imgNames.size()-1);
	for(size_t i=0;i<imgNames.size()-1;++i) corrName[i]=imgNames[i]+".tsk.mtchc";

	vector<vector<pair<int,int> > > correpss;

	correpss.resize(corrName.size());

	for (size_t i = 0; i < correpss.size(); ++i)
	{
		fileIOclass::InVector(corrName[i],[](FILE* fp,pair<int,int>& w){fscanf(fp,"%d %d",&w.first,&w.second);},correpss[i]);
	}
	vector<vector<vector<int> > >  feas(feaNames.size());

	for(size_t i=0;i< feas.size();++i)
	{
		fileIOclass::InVector(feaNames[i],[](FILE* fp,vector<int>& fea)
		{
			fea.resize(2);
			fscanf(fp,"%lf %lf\n",&fea[0],&fea[1]);
			
		},feas[i]);
	}

	auto drawsimple=[&](const string& imgn,int inde)
	{
		Mat img=imread(imgn,-1);
		for(int j=max(0,inde-5);j<=inde;++j)
		{
			for(int k=0;k<feas[j].size();++k)
			{
				Point2f p(feas[j][k][0],feas[j][k][1]);
				circle( img,
				 p,
				 1,
				 Scalar(0,255,255),
				 2,
				 8 );
			}
		}

		//if(inde<1) return;
		for(int j= max(0,inde-4);j<inde && j<correpss.size() ;++j)
		{
			auto& w=correpss[j];
			for(int k=0;k<w.size();++k)
			{
				auto& mw=w[k];
				Point2f p1(feas[j][mw.first][0],feas[j][mw.first][1]);
				Point2f p2(feas[j+1][mw.second][0],feas[j+1][mw.second][1]);
				line(img,p1,p2,Scalar(255,0,0), 1, CV_AA);
			}
		}

		imwrite("w_"+imgn,img);
	};
	for(size_t i=0;i<imgNames.size();++i)
	{
		drawsimple(imgNames[i],i);
	}

	vector<vector<Point2f> > features(feas.size());
	vector<map<int,int> > sth(correpss.size());

	for (int i = 0; i < feas.size(); i++)
	{
		features[i].resize(feas[i].size());
		for (int j = 0; j < feas[i].size(); j++)
		{

			features[i][j]=Point2f(feas[i][j][0],feas[i][j][1]);
		}
	}
	for(int i=0;i<correpss.size();++i)
	{
		for(int j=0;j<correpss[i].size();++j)
			sth[i][correpss[i][j].first]=correpss[i][j].second;
	}
	auto trajs=incrementalTrajectoryDetect(features,sth);
	

	vector<bool> lbs(trajs.size(),false);
	for (int i = 0; i < trajs.size(); i++)
	{
		if(trajs[i][0].x>0.1)
			lbs[i]=true;
	}

	

	removeStaticTrajectories(trajs);



	
	vector<vector<Point2f> > effect_trajs(trajs.size());

#pragma omp parallel for
	for (int i = 0; i < trajs.size(); i++)
	{
		cout<<i<<"\t";
		effect_trajs[i]= effectTraj (trajs[i]);
	}

	Mat img=imread(imgNames[0],-1);

	saveTraj_BannoFormat("trajj.txt",imgNames,trajs,img);
}
*/


int main0818(int argc, char* argv[])
{
	//_chdir("D:\\DATA\\campodia_new\\sfm");
	_chdir("D:\\DATA\\ttwoladybug\\ladybug\\2s");
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

int main886(int argc,char* argv[])
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

int mainShit()
{
	_chdir("D:\\DATA\\0717Seiken");

	string fnm="Seiken_E.ptx";


	vector<vector<vector<double> > > laser_sc_lns;
	int pnt_per_ln,laser_lns;

	FILE* fp=fopen(fnm.c_str(),"r");
	fscanf(fp,"%d\n%d\n",&laser_lns,&pnt_per_ln);

	FILE* ft;
	ft=fopen("down.ptx","w");
	//vector<vector<vector<double> > > down(laser_lns/4,vector<vector<double> > (pnt_per_ln/10,vector<double>(3)));
	fprintf(ft,"%d\n%d\n",laser_lns/4,pnt_per_ln/4);

	double tem;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			fscanf(fp,"%lf ",&tem);
			fprintf(ft,"%lf ",tem);
			

		}
		fscanf(fp,"\n");
		fprintf(ft,"\n");
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			fscanf(fp,"%lf ",&tem);
			fprintf(ft,"%lf ",tem);
		}
		fprintf(ft,"\n");
		fscanf(fp,"\n");
	}

	laser_sc_lns.resize(laser_lns,vector<vector<double> > (pnt_per_ln,vector<double>(3)));

	int a,b,c;
	

	for (int i = 0; i < laser_lns; i++)
	{
		cout<<i<<"\t";
		for (int j = 0; j < pnt_per_ln; j++)
		{
			fscanf(fp,"%lf %lf %lf %lf %d %d %d\n",&laser_sc_lns[i][j][0],&laser_sc_lns[i][j][1],&laser_sc_lns[i][j][2],&tem,&a,&b,&c);
			if((i%4==0)&&(j%4==0)) 
				fprintf(ft,"%lf %lf %lf %lf\n",laser_sc_lns[i][j][0],laser_sc_lns[i][j][1],laser_sc_lns[i][j][2],tem);
		}
	}


	fclose(fp);


	
	fclose(ft);
	return 1;
}

int main9(int argc, char* argv[])
{
	_chdir("D:\\DATA\\ttwoladybug\\ladybug\\tss");

	string s="tmat.lst";

	auto fns=fileIOclass::InVectorString(s);

	vector<vector<double> > po(fns.size(),vector<double>(3,0.0));

	for(size_t i=0;i<fns.size();++i)
	{
		FILE* fp=fopen((fns[i]+".trans").c_str(),"r");
		for(int j=0;j<3;++j)
			fscanf(fp,"%lf ",&po[i][j]);
		fclose(fp);
	}

	for(size_t i=1;i<po.size();++i)
	{
		for(int j=0;j<3;++j)
			po[i][j]+=po[i-1][j];
	}

	for(size_t i=0;i<po.size();++i)
	{
		for(size_t j=0;j<3;++j)
			cout<<po[i][j]<<" ";
		cout<<endl;
	}

	return 0;
}

int main134(int argc, char* argv[])
{
	_chdir("F:\\tss");

	string s("ladybug_panoramic_001940.jpg.tsk");

	if(argc>1)
		s=argv[1];

	auto fnms=fileIOclass::InVectorString(s);
	vector<vector<int> > f1,f2,match;
	f1=fileIOclass::InVectorSInt(fnms[0]+".orbc",2);
	f2=fileIOclass::InVectorSInt(fnms[1]+".orbc",2);
	match=fileIOclass::InVectorSInt(s+".mtchc",2);

	vector<vector<double> > sph1,sph2;
	sph1.resize(f1.size(),vector<double>(3));
	sph2.resize(f2.size(),vector<double> (3));

	for(size_t i=0;i<f1.size();++i)
		sph1[i]=Img2Serph((double)f1[i][0],(double)f1[i][1],512,256);
	for(size_t i=0;i<f2.size();++i)
		sph2[i]=Img2Serph((double)f2[i][0],(double)f2[i][1],512,256);


	auto func=[](vector<double> a,vector<double> b)->vector<double>
	{
		vector<double> rslt(9,0);

		for(size_t i=0;i<3;++i)
			for(size_t j=0;j<3;++j)
				rslt[i*3+j]=a[i]*b[j];

		return rslt;
	};

	string oun=s+".tmatc";
	FILE* fp=fopen(oun.c_str(),"w");
	for(size_t i=0;i<match.size();++i)
	{
		int i1=match[i][0];
		int i2=match[i][1];

		auto tt1=f1[i1];auto tt2=f2[i2];
		auto tt3=sph1[i1];auto tt4=sph2[i2];

		auto w=func(sph1[i1],sph2[i2]);
		for(size_t j=0;j<9;++j)
			fprintf(fp,"%lf ",w[j]);
		fprintf(fp,"\n");
	}

	fclose(fp);


	return 0;
}


int main443(int argc, char* argv[])
{
//	_chdir("D:\\DATA\\campodia_new\\fm");

#ifdef _DEBUG
	_chdir("E:\\results\\ssf");
#endif 



	_chdir("D:\\DATA\\ttwoladybug\\ladybug\\tss");

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

	saveTraj_BannoFormat("trajj.txt",flnms,trajs,img);
	
//	waitKey();
	//testsimple(imgs[0],imgs[1]);
	
	return 1;
}