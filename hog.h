#pragma once 
#include <vector>
#include <algorithm>
using namespace std;
#define bin_num 9
#define block_width 20
#define block_height 20
vector<double> hogFeatureSpecial(const vector<vector<int> >& grayImage)
{

	int block_h_num=grayImage.size()/block_height;
	int block_w_num=grayImage.size()/block_width;
	vector<double> rslt(bin_num*block_h_num*block_w_num,0.0);
	static double bin_div[bin_num-1]={-2.7474	,
				-1.1917	,
				-0.5773	,
				-0.1763	,
				0.1763	,
				0.5773	,
				1.1917	,
				2.7474	
				};
	double img_Size=(grayImage.size()-2)*(grayImage[0].size()-1);
	for (int i = 0; i < block_h_num; i++)
	{
		for (int j = 0; j < block_w_num; j++)
		{
			
			for (int k = 0; k < block_height; k++)
			{
				for (int l = 0; l <block_width ; l++)
				{
					int y=block_height*i+k;
					int x=block_width*j+l;
					auto test =[](int a,int b,int h,int w)->bool
					{
						return (a>0)&&(a<(h-1))&&(b>0)&&(b<w-1);
					};
					if(test(y,x,grayImage.size(),grayImage[0].size()))
					{
						double wh=grayImage[y+1][x]-grayImage[y-1][x];
						double ww=grayImage[y][x+1]-grayImage[y][x-1];
						if(abs(ww)< 0.0001) ww=ww>0?0.0001:-0.0001;
						double value=wh/ww;
						int bin_index=-1;
						for (int m = 0; m < bin_num-1; m++)
						{
							if (value<bin_div[m])
							{
								bin_index=m;
								break;
							}
						}
						if(bin_index==-1)
							bin_index=bin_num-1;
						rslt[(block_w_num*i+j)*bin_num+bin_index]+=1.0;
					}
				}
			}
		}
	}
	for(auto&sth:rslt) sth/=img_Size;

	return rslt;
}

/*
class hog
{
public:
	void init(int height,int width,IplImage* RGBimg)
	{
		double s=-3.14159/2;
		for (int i=1;i<bin_num;i++)
		{

			bin_div[i-1]=tan(s+i*3.14159/bin_num);
		}

		IplImage * img=cvCreateImage(cvSize(RGBimg->width,RGBimg->height),IPL_DEPTH_8U,1);
		cvCvtColor(RGBimg, img, CV_BGR2GRAY);
		grayImg.resize(height,vector<int>(width,0));
		for (int i=0;i<height;i++)
		{

			for (int j=0;j<width;j++)
			{
				CvScalar s;
				s=cvGet2D(img, i, j);
				grayImg[i][j]=s.val[0];
			}
		}
		cvReleaseImage(&img);
	}
	bool Cell(vector<vector<int>> img,int x,int y,double* feature_)
	{


		for(int i=0;i<bin_num;i++)
		{
			feature_[i]=0;
		}
		for (int i=y+1;i<y+cell_height-1;i++)
		{
			for (int j=x+1;j<x+cell_width-1;j++)
			{
				//cout<<atan(-1.0)*180/3.1415926;
				double zi,mu;
				zi=img[i+1][j]-img[i-1][j];
				mu=img[i][j+1]-img[i][j-1]+0.01;
				double value=zi/mu;
				int index;
				for (int k=0;k<bin_num-1;k++)
				{
					if (value<=bin_div[0])
					{
						index=0;break;
					}
					else if (value>=bin_div[bin_num-2])
					{
						index=bin_num-1;break;
					}
					else if (value>=bin_div[k]&&value<=bin_div[k+1])
					{
						index=k+1;
						break;
					}
				}
				index-=(int)(bin_num/2);
				if (index<0)
				{
					index+=bin_num;
				}

				//cout<<(int)value/180*bin_num<<endl;
				feature_[index]+=1;
			}
		}
		double tem=0.01;
		for (int i=0;i<bin_num;i++)
		{
			tem+=feature_[i];
		}
		for (int i=0;i<bin_num;i++)
		{
			feature_[i]=sqrt(feature_[i]/tem);
		}
		return true;
	}
	bool HOG(vector<double> &feature,vector<vector<int>> img,int startX,int startY)
	{
	

		int imgW=img[0].size();
		int imgH=img.size();

		if (imgW==0||imgH==0)
		{
			return false;
		}
		if ((startX+block_width*cell_width>=imgW+1)||(startY+block_height*cell_height>=imgH+1))
		{
			return false;
		}

		double* temp;
		temp=new double[bin_num];
		feature.clear();

		

		int inde=0;

		for (int i=0;i<block_height;i++)
		{
			for (int j=0;j<block_width;j++)
			{

				//cout<<"("<<i<<","<<j<<")";
				Cell(img,startX+j*cell_width,startY+j*cell_height,temp);

				for (int k=0;k<bin_num;k++)
				{
					feature.push_back(temp[k]);
				}

			}
		}



		return true;

	}
protected:
private:
};


*/

