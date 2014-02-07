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

template<class PtType>
void PointsToLine(const vector<PtType>& traj,double& a,double &b,double  & r)
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

void drawPoint(Mat& img,const Point& pt)
{
	int thickness = -1;
	int lineType = 8;
	circle( img,
         pt,
         4,
         Scalar( 0, 255, 255 ),
         thickness,
         lineType );
}

void drawPoints(Mat& img,const vector<Point>& pts)
{
	
 
	for(const auto& pt:pts)
		drawPoint(img,pt);
		
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
void drawHlines( Mat& src,const vector<Vec4i>& lines,vector<bool> lbs=vector<bool>(0),bool showfalse=true)
{
	for(size_t i=0;i<lines.size();i++)
	{
		auto& l=lines[i];
		if(lbs.size()==0)
			line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
		else if(showfalse)
			line( src, Point(l[0], l[1]), Point(l[2], l[3]), lbs[i]?Scalar(0,0,255):Scalar(255,0,0), 1, CV_AA);
		else if(lbs[i])
			line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
	}
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
	drawHlines(src,lines);
	
	imshow("source", src);
	return lines;
}

void drawTrajs(Mat& img,const vector<vector<Point2f> >& trjs,vector<bool> lbs=vector<bool>(0),bool showfals=true)
{
	for (int i = 0; i < trjs[0].size(); i++)
	{
		for (int j = 0; j < trjs.size()-1; j++)
		{
			if(lbs.size()==0)
				line(img,trjs[j][i], trjs[j+1][i], Scalar(255,0,0), 2, CV_AA);
			else if(showfals)
				line(img,trjs[j][i], trjs[j+1][i], lbs[i]?Scalar(0,255,255):Scalar(255,255,0), 2, CV_AA);
			else if(lbs[i])
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

Point intersectionPoint(pair<double,double> fir,pair<double,double> sec)
{
	double db=sec.second-fir.second;
	double ac=fir.first-sec.first;
	if(ac<0.0001)
		ac=0.0001;

	double x=db/ac;
	double y=fir.first*x+fir.second;

	return Point(x,y);
}

Point vanishFromSky(const Mat& img)
{
	auto sth=skyDetect(img);
	return intersectionPoint(sth[0],sth[1]);
}

void maxChange(int index,const vector<Point> pts,bool& turnoff)
{
//	double maxDis=0.0;
	for (int i = index; i >0; i--)//no need to trace along so much, I hate this
	{

		double ds=norm(pts[i]-pts[i-1]);
		if(ds>skyVanishPtChg_limit)
		{
			turnoff=true;
			break;
		}
	}
}

void tjsTolns(vector<tuple<double,double,double> >& linesOfTrjs,const vector<vector<Point2f>> & trjs)
{
	for (size_t i = 0; i < linesOfTrjs.size(); i++)
	{
		auto& sth=linesOfTrjs[i];
		vector<Point2f> trj;
		for (int j = 0; j < trjs.size(); j++)
		{
			trj.push_back(trjs[j][i]);
		}
		PointsToLine(trj,get<0>(sth),get<1>(sth),get<2>(sth));
	}
}
template<class pttype>
double pointToLineDis(double a,double b,const pttype& pt)
{
	if(abs(a)<0.0001)
		a=(a>0)?0.0001:-0.001;
	double x=b/a;
	double y=b;
	return abs((a*pt.x+b-pt.y)*x/sqrt(x*x+y*y));
}
template<class pttype>
double pointToHlineDis(const Vec4i& hline,const pttype& pt)
{
	int x1(hline[0]),y1(hline[1]);//hline[0],hline[1]
	double d1=norm(pt-pttype(x1,y1));
	int x2(hline[2]),y2(hline[3]);
	double d2=norm(pt-pttype(x2,y2));
	double t1=x2-x1; if (abs(t1)<0.001) t1= t1>0?0.001:-0.001;
	double a=(double)(y2-y1)/t1;
	double b=y1-a*x1;
	double d3=pointToLineDis(a,b,pt);

	return max(d3,min(d1,d2));
	//line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
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
void HlineTrajCorr(const vector<Point2f>& pts,const Vec4i& hline,vector<int>& rslt )
{
	vector<double> diss(pts.size());
	vector<int> index(pts.size());
	for (int i = 0; i < pts.size(); i++)
	{
		diss[i]=pointToHlineDis(hline,pts[i]);
		index[i]=i;
	}
	FromSmall(diss,diss.size(),index);
	for (int i = 0; i < rslt.size(); i++)
	{
		rslt[i]=index[i];
	}
	
}

Point vanishPointDecide(const Mat& img,int index,const vector<Point> & skyVpts,
	const vector<Vec4i> & hlines,const vector<vector<Point2f> > & trjs)
{
	Point rslt;

	Mat copy=img.clone();
	
	
	drawPoint(copy,skyVpts[index]);
	

	static bool skyVpt_turnoff=false;//notice, this is a in-function static variable, though I dislike this a lot
	if(!skyVpt_turnoff)
	{
		maxChange(index,skyVpts,skyVpt_turnoff);
	}
	vector<tuple<double,double,double> > linesOfTrjs(trjs[0].size());//this is dirty and quick [0]
	tjsTolns(linesOfTrjs,trjs);

	vector<double> trjsLenth(trjs[0].size());
	for (int i = 0; i < trjsLenth.size(); i++)
	{
		trjsLenth[i]=norm(trjs[0][i]-trjs[trjs.size()-1][i]);
	}

	vector<bool> trjlbs(trjs[0].size(),true);
	vector<bool> hlnlbs(hlines.size(),true);

	for (int i = 0; i < trjs[0].size(); i++)//dirty and quick
	{
		if((get<2>(linesOfTrjs[i])<trajectoryIsLine_threshold)||(trjsLenth[i]<trajectoryLength_constrain))
			trjlbs[i]=false;
	}
	vector<vector<int> > hlineTrjCorres(hlines.size(),vector<int>(TrjNumOfEachLine));

	vector<pair<double,double> > abln_hlines(hlines.size());

	for (int i = 0; i < hlines.size(); i++)
	{
		double x=hlines[i][2]-hlines[i][0];
		double y=hlines[i][3]-hlines[i][1];

		int x1(hlines[i][0]),y1(hlines[i][1]);//hline[0],hline[1]
	//	double d1=norm(pt-pttype(x1,y1));
		int x2(hlines[i][2]),y2(hlines[i][3]);
	//	double d2=norm(pt-pttype(x2,y2));
		double t1=x2-x1; if (abs(t1)<0.001) t1= t1>0?0.001:-0.001;
		double a=(double)(y2-y1)/t1;
		double b=y1-a*x1;
		abln_hlines[i].first=a;
		abln_hlines[i].second=b;

		if(abs(x)<0.001) x=0.001;
		double r=abs(y/x);
		if((r>houghXYRatio_consgtrain)||(r<1.0/houghXYRatio_consgtrain))
			hlnlbs[i]=false;
		HlineTrajCorr(trjs[0],hlines[i],hlineTrjCorres[i]);

		double maxdis=0.0;
		for (int j = 0; j < hlineTrjCorres[i].size(); j++)
		{
			auto dis=trjsLenth[hlineTrjCorres[i][j]];
			if(dis>maxdis)
				maxdis=dis;
		}
		if (maxdis<10.0)
			hlnlbs[i]=false;

	}

	vector<Point> candidates;

	if (!skyVpt_turnoff)
	{
		int middlex=skyVpts[index].x;
		int middley=skyVpts[index].y;
		for (int x=middlex-halfSearchSizeX/4;x<middlex+halfSearchSizeX/4;x+=firstSearchStepx/2)
		{
			for (int y = middley-halfSearchSizeY/4; y < middley+halfSearchSizeY/4; y+=firstSearchStepy/2)
			{
				candidates.push_back(Point(x,y));
			}
		}
	}
	else
	{
		int middlex=img.cols/2;
		int middley=img.rows/2;
		for (int x=middlex-halfSearchSizeX;x<middlex+halfSearchSizeX;x+=firstSearchStepx)
		{
			for (int y = middley-halfSearchSizeY; y < middley+halfSearchSizeY; y+=firstSearchStepy)
			{
				candidates.push_back(Point(x,y));
			}
		}
	}

	

	vector<double> candidateConfidence(candidates.size(),0.0);

	double largeConfi=0.0;
	int largindex=0;

	for (int i = 0; i < candidates.size(); i++)
	{
		
		for (int j = 0; j < linesOfTrjs.size(); j++)
		{
			if(trjlbs[j])
			{
				if(pointToLineDis(get<0>(linesOfTrjs[j]),get<1>(linesOfTrjs[j]),candidates[i])<candidateSelectThres_traj)
					candidateConfidence[i]+=traj_weight;
				
			}
		}

		for (int j = 0; j < abln_hlines.size(); j++)
		{
			if(hlnlbs[j])
			{
				if(pointToLineDis(abln_hlines[j].first,abln_hlines[j].second,candidates[i])<candidateSelectThres_hline)
					candidateConfidence[i]+=hline_weight;
			}
		}
		if (candidateConfidence[i]>largeConfi)
		{
			largeConfi=candidateConfidence[i];
			largindex=i;
		}
	}

	drawHlines(copy,hlines,hlnlbs);
	drawTrajs(copy,trjs,trjlbs);
	imshow("rslt",copy);

	copy=img.clone();


	drawHlines(copy,hlines,hlnlbs,false);
	drawTrajs(copy,trjs,trjlbs,false);
	drawPoint(copy,candidates[largindex]);
	imshow("newrslt",copy);

	return candidates[largindex];
}