#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"

using namespace std;



int main(int argc, char* argv[])
{
	_chdir("E:\\vanish\\dataset\\paris");

	auto flnms=fileIOclass::InVectorString("img.lst");

	vector<Mat> imgs(flnms.size());
	vector<Point> skyVanish(flnms.size());
	vector<Point> vnspts(flnms.size());
	for(size_t i=0;i<flnms.size();++i)
	{
		imgs[i]=imread(flnms[i]+".jpg");

//		skyDetect(imgs[i]);
		skyVanish[i]=vanishFromSky(imgs[i]);
		auto lines=houghLine(imgs[i]);
		if(i>trajectoryL)
		{
			auto trjs=trajectoryDetect(imgs,(int)i);

			vnspts[i]=vanishPointDecide(imgs[i],i,skyVanish,lines,trjs);
			int x=0;int y=0;int count=0;
			for (int j = trajectoryL+1; j <=i; j++)
			{
				x+=vnspts[j].x;
				y+=vnspts[j].y;
				count++;
			}
			Mat copy=imgs[i].clone();
			int thickness = -1;
			int lineType = 8;
			circle( copy,
			 Point(x/count,y/count),
			 4,
			 Scalar( 0, 255, 255 ),
			 thickness,
			 lineType );
			imshow("final_reslt",copy);
			waitKey();
		}

		
		
		
		
		
	}
	return 0;
}