#include <Windows.h>
#include <direct.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "Vanish.h"

using namespace std;



int main(int argc, char* argv[])
{
	_chdir("E:\\vanish\\dataset\\ny");

	auto flnms=fileIOclass::InVectorString("img.lst");

	vector<Mat> imgs(flnms.size());
	

	for(size_t i=0;i<flnms.size();++i)
	{
		imgs[i]=imread(flnms[i]+".jpg");
		if(i>trajectoryL)
			trajectoryDetect(imgs,(int)i);
		
		skyDetect(imgs[i]);
		houghLine(imgs[i]);
		waitKey();
	}
	return 0;
}