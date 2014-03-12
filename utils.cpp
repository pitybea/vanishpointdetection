
#include "utils.h"





void drawTrajs_ok(Mat& img,const vector<vector<Point2f> >& trjs,vector<bool> lbs,bool showfals)
{
	for (int j = 0; j < trjs.size(); ++j)
	{
		
		if(trjs[j].size()>1)
		for (int i = 0; i < trjs[j].size()-1; i++)
		{
			if(lbs.size()==0)
				line(img,trjs[j][i], trjs[j][i+1], Scalar(255,0,0), 1, CV_AA);
			else if(showfals)
				line(img,trjs[j][i], trjs[j][i+1], lbs[i]?Scalar(0,255,255):Scalar(255,255,0), 1, CV_AA);
			else if(lbs[i])
				line(img,trjs[j][i], trjs[j][i+1], Scalar(255,0,0), 1, CV_AA);
		}
	}
}




void drawTrajs(Mat& img,const vector<vector<Point2f> >& trjs,vector<bool> lbs,bool showfals)
{
	for (int i = 0; i < trjs[0].size(); i++)
	{
		for (int j = 0; j < trjs.size()-1; j++)
		{
			if(lbs.size()==0)
				line(img,trjs[j][i], trjs[j+1][i], Scalar(255,0,0), 1, CV_AA);
			else if(showfals)
				line(img,trjs[j][i], trjs[j+1][i], lbs[i]?Scalar(0,255,255):Scalar(255,255,0), 1, CV_AA);
			else if(lbs[i])
				line(img,trjs[j][i], trjs[j+1][i], Scalar(255,0,0), 1, CV_AA);
		}
	}
}