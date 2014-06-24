
#include  <vector>

#include <random>
#include <map>
#include <tuple>
#include <math.h>

using namespace std;

/*vector<double> Img2Serph(double x,double y)
{
	double alpha= -x*2*pi/8000+2*pi;
	double beta=  -pi*y/4000+pi/2;

	vector<double> rslt(3);
	rslt[0]=cos(beta)*cos(alpha);
	rslt[1]=cos(beta)*sin(alpha);
	rslt[2]=sin(beta);
	return rslt;
}*/



auto genSomeData(double small,double large,double stepSmall,double stepLarge,int pointNumber,int steps )->tuple<vector<vector<double> >,vector<vector<double> >, vector<vector<vector<double> > > >
{
	vector<vector<double> > positions(steps,vector<double>(3,0.0));
	vector<vector<double> >  points(pointNumber,vector<double>(3,0.0));
	vector<vector<vector<double> > > observisions(steps,vector<vector<double> > (pointNumber,vector<double>(3,0.0)));

//	positions.push_back(vector<double>(3,0.0));

	uniform_real_distribution<double> unif(small,large);

	default_random_engine re;
	for (int i = 0; i < pointNumber; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			points[i][j]=unif(re);
		}
	}

	uniform_real_distribution<double> unif2(stepSmall,stepLarge);

	for (int i = 1; i < steps; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			positions[i][j]=positions[i-1][j]+unif2(re);
		}
	}

	auto pntfunc=[](vector<double>& center, vector<double>& pnt,vector<double> des)
	{
		//assert(center.size()==3);
		double sum=0.0;
		vector<double> t(3);
		for(int j=0;j<3;++j)
		{
			t[j]=pnt[j]-center[j];
			sum+=t[j]*t[j];
		}
		double r=sqrt(sum);

		for(int j=0;j<3;++j)
		{
			des[j]=center[j]+t[j]/r;
		}
		
	};


	for (int i = 0; i < steps; ++i)
	{
		for (int j = 0; j < pointNumber; ++j)
		{
			pntfunc(positions[i],points[j],observisions[i][j]);
		}
	}

	return tuple<vector<vector<double> >,vector<vector<double> >, vector<vector<vector<double> > > >(positions,points,observisions);

}