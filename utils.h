#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "Vsettings.inl"
using namespace cv;

void drawTrajs(Mat& img,const vector<vector<Point2f> >& trjs,vector<bool> lbs=vector<bool>(0),bool showfals=true);
void drawTrajs_ok(Mat& img,const vector<vector<Point2f> >& trjs,vector<bool> lbs=vector<bool>(0),bool showfals=true);
template<class PtType>
void drawPoint(Mat& img,const PtType& pt,int siz=4)
{
	int thickness = 1;
	int lineType = 8;
	circle( img,
         Point(pt.x,pt.y),
         siz,
         Scalar( 0, 255, 255 ),
         thickness,
         lineType );
}


template<class PtType>
void drawPoints(Mat& img,const vector<PtType>& pts,int siz=4)
{
	
 
	for(const auto& pt:pts)
		drawPoint(img,pt,siz);
		
}

template<class T>
static void FromSmall(vector<T>& p,int n,vector<int>& index)
{
	int k,j,i;
	T t;
	int ii;
	k=n/2;
	while(k>0)
	{
		for(j=k;j<=n-1;j++)
		{
			t=p[j];  ii=index[j];  i=j-k;
			while((i>=0)&&(p[i]>t))
			{
				p[i+k]=p[i];  index[i+k]=index[i];  i=i-k;
			}
			p[i+k]=t;  index[i+k]=ii;
		}
		k=k/2;
	}
};