#include "trainTest.h"

inline int grayValue(const Mat& grayimg,int x,int y)
{
	return static_cast<int>(grayimg.at<uchar>(Point(x,y)));
}
bool evaluateByCanny(const Mat& cimg,const Point& pt)
{
	int rslt=0;
	int start_x=pt.x-traintest::Example_width/2;
	int start_y=pt.y-traintest::Example_height/2;
	for (int i = start_x; i < start_x+traintest::Example_width; i++)
	{
		for (int j = start_y; j < start_y+traintest::Example_height; j++)
		{
			if(grayValue(cimg,i,j)==1) ++rslt;
		}
	}
	int area=traintest::Example_width*traintest::Example_height;

	return ((double)rslt/area)>traintest::edge_ratio;

	//return rslt;
}

bool inImage(int width,int height,const Point& pt)
{
	return (pt.x>1)&&(pt.x<(width-1))&&(pt.y>1)&&(pt.y<(height-1));
}
void SampleExamples(const Mat& img, vector<Point>& trainposs,vector<Point>& testposs,
					vector<Mat>& trainimgs, vector<Mat>& testimgs)
{
	 static std::default_random_engine generator(::time(0));
     std::uniform_real_distribution<float> distribution(0.0, 1);

	 Mat src=img.clone(),dst;
	 blur(src,src,Size(xBlurSize,yBlursize));
	 Canny(src, dst, cannyLowThreshold, cannyLargeThreshold, cannyApertureSize);
	 int img_width=img.cols;
	 int img_height=img.rows;

	 int train_num=0;
	 int test_num=0;

	 auto area=[=](Point pt)->Rect{int wth=traintest::Example_width/2;int ht=traintest::Example_height/2;
	 return Rect(pt.x-wth,pt.y-ht,wth*2,ht*2);};
	 while(train_num<traintest::Train_example_num)
	 {
		 Point pt((double)img_width/2*distribution(generator),
			 (double)img_height*distribution(generator));
		 if(inImage(img_width,img_height,pt)&&evaluateByCanny(dst,pt))
		 {
			 ++train_num;
			 trainposs.push_back(pt);
			 trainimgs.push_back(Mat(src,area(pt)));
		 }

	 }
	 while(test_num<traintest::Test_example_num)
	 {
		 Point pt((double)img_width/2*distribution(generator)+img_width/2,
			 (double)img_height*distribution(generator));
		 if(inImage(img_width,img_height,pt)&&evaluateByCanny(dst,pt))
		 {
			 ++test_num;
			 testposs.push_back(pt);
			 testimgs.push_back(Mat(src,area(pt)));
		 }
	 }




}