#include "Vanish.h"
inline int grayValue(const Mat& grayimg,int x,int y)
{
	return static_cast<int>(grayimg.at<uchar>(Point(x,y)));
}

void findStartX(const Mat& img,int &x,int y)
{
	int width=img.cols-1;
	int maxindex=0;
	int maxvalue=0;

	int startx=0;
	while(startx<width)
	{
		if(grayValue(img,startx,y)==0)
		{
			int rightx(startx);


			while((rightx<width)&&(grayValue(img,++rightx,y)==0))
				;
			int dis=rightx-startx;
			if (dis>maxvalue)
			{
				maxindex=startx;
				maxvalue=dis;
			}

			startx=rightx;
		}
		else
			startx+=1;
	}
	x=maxindex+maxvalue/2;
}

void findSkyFromCannyImage(const Mat& cannyimg,vector<Point> (&edges)[2])
{
	int width=cannyimg.cols-1;
	int height=cannyimg.rows;

	int currenty=2;
	int currentx;
	findStartX(cannyimg,currentx,currenty);

//	auto sth=static_cast<int> (cannyimg.at<uchar>(Point(0,0)));

	while (( grayValue(cannyimg,currentx,currenty) ==0 )&&(currenty<height))
	{
		int leftx(currentx),rightx(currentx);
		while((leftx>1)&&(grayValue(cannyimg,--leftx,currenty)==0))
			;
		while((rightx<width)&&(grayValue(cannyimg,++rightx,currenty)==0))
			;		
		
		edges[0].push_back(Point(leftx,currenty));
		edges[1].push_back(Point(rightx,currenty));
		
		currentx=(leftx+rightx)/2;

		++currenty;

	}
	
}


void PointsToLine(const vector<Point>& traj,double& a,double &b,double  & r)
{
	a=0.0;
	b=0.0;
	r=0.0;
	double sumx(0.0),sumy(0.0),sumxy(0.0),sumx2(0.0),sumy2(0.0);
	if (traj.size()>2)
	{
		size_t n=traj.size();
		
		for (const auto &pt: traj)
		{
			sumx+=pt.x;
			sumy+=pt.y;
			sumxy+=pt.x*pt.y;
			sumx2+=pt.x*pt.x;
			sumy2+=pt.y*pt.y;

		}

		a=(sumxy*n-sumy*sumx)/(sumx2*n-sumx*sumx);
		b=sumy/n-a*sumx/n;
		double xAv(sumx/n),yAv(sumy/n);
		double sumXav(0.0),sumYav(0.0),sumXyav(0.0);
		for (const auto& pt :traj)
		{
			sumXav+=(pt.x-xAv)*(pt.x-xAv);
			sumYav+=(pt.y-yAv)*(pt.y-yAv);
			sumXyav+=(pt.x-xAv)*(pt.y-yAv);
		}
		r=abs(sumXyav/(sqrt(sumXav*sumYav)));
	
	}

}

void drawABLine(Mat& img,double a,double b)
{
	Point pt1(-1000,(int)(-a*1000)+b),pt2(1000,(int)(a*1000+b));
	line(img,pt1, pt2, Scalar(255,0,0), 2, CV_AA);
}
void drawPoints(Mat& img,const vector<Point>& pts)
{
	int thickness = -1;
	int lineType = 8;
 
	for(const auto& pt:pts)
		circle( img,
         pt,
         1,
         Scalar( 0, 0, 255 ),
         thickness,
         lineType );
		
}

auto skyDetect(const Mat& osrc)->vector<pair<double,double> >
{
	vector<pair<double,double> > rslt(2);

	Mat dst, cdst;
	Mat src=osrc.clone();
	blur(src,src,Size(xBlurSize,yBlursize));
	Canny(src, dst, cannyLowThreshold, cannyLargeThreshold, cannyApertureSize);

	vector<Point> skyEdges[2];
	findSkyFromCannyImage(dst,skyEdges);


	cvtColor(dst,cdst,CV_GRAY2BGR);

	auto& ab=rslt.begin();
	for(auto& sth:skyEdges)
	{
		double r;
		if(sth.size()>30)
			PointsToLine(vector<Point>(sth.begin()+30,sth.end()),(*ab).first,(*ab).second,r);
		else
			PointsToLine(vector<Point>(sth.begin(),sth.end()),(*ab).first,(*ab).second,r);


		drawABLine(cdst,(*ab).first,(*ab).second);
		drawPoints(cdst,sth);
		++ab;
	}

	imshow("detected lines", cdst);
	return rslt;
	
}

auto houghLine(const Mat& osrc)->vector<Vec4i>
{
	Mat dst, cdst;
	Mat src=osrc.clone();
	blur(src,src,Size(xBlurSize,yBlursize));
	Canny(src, dst, cannyLowThreshold, cannyLargeThreshold, cannyApertureSize);
	cvtColor(dst,cdst,CV_GRAY2BGR);

	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, houghThreshold, houghMinLineLength, houghMaxLineGap );

	for(auto& l:lines)
	{
		line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
	}
	imshow("source", src);
	return lines;
}

void drawTrajs(Mat& img,const vector<vector<Point2f> >& trjs)
{
	for (int i = 0; i < trjs[0].size(); i++)
	{
		for (int j = 0; j < trjs.size()-1; j++)
		{
			line(img,trjs[j][i], trjs[j+1][i], Scalar(255,0,0), 2, CV_AA);
		}
	}
}

auto trajectoryDetect(const vector<Mat>& imgs,int index)->vector<vector<Point2f> > 
{

		const Mat &src=imgs[index];
		Mat src_gray;
		
		cvtColor( src, src_gray, CV_BGR2GRAY );
		vector<vector<Point2f> > cornerss(trajectoryL,vector<Point2f>(kptDet_maxCorners));
		
		Mat copy;
		copy = src.clone();

		goodFeaturesToTrack( src_gray,
				   cornerss[0],
				   kptDet_maxCorners,
				   kptDet_qualityLevel,
				   kptDet_minDistance,
				   Mat(),
				   kptDet_blockSize,
				   kptDet_useHarrisDetector,
				   kptDet_k );

		//vector<char> featuresFound;
       // Mat err;
		 vector<uchar> status;
		 vector<float> err;
		 TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, kptTrack_iter, kptTrack_epsin);

		 for (int i = 0; i < trajectoryL-1; i++)
		 {
			 calcOpticalFlowPyrLK(imgs[index-i],imgs[index-i-1],cornerss[i],cornerss[i+1],status,err,Size(kptTrack_winsize,kptTrack_winsize),kptTrack_maxlevel, termcrit, 0, 0.001);
		 }

		

		 drawTrajs(copy,cornerss);


		imshow( "what ever", copy );
		return cornerss;
		

	

}