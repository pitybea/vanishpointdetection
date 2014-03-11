#include "Traj.h"
#include <thread>
#include <map>
#include "munkres.h"
using namespace traj;
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



auto incrementalTrajectoryDetectCore(const vector<Mat>& imgs)-> pair<vector< pair<vector<Point2f> ,vector<Point2f> > >, vector<map<size_t,size_t>>>
{
	assert(imgs.size()>=2);

	vector<pair<vector<Point2f> ,vector<Point2f> > > features(imgs.size()-1);
	
	vector<map<size_t,size_t> > corres(imgs.size()-2);

	for (size_t i=0;i<imgs.size()-1;++i)
	{
		features[i].first.reserve(kptDet_maxCorners);
		features[i].second.reserve(kptDet_maxCorners);
	}



	size_t thr_num1=imgs.size()-1;

	thread* thrds1=new thread[thr_num1];

	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, kptTrack_iter, kptTrack_epsin);

	for (size_t i = 0; i < thr_num1; i++)
	{
		thrds1[i]=thread([&](const Mat& fir,const Mat& sec,pair<vector<Point2f> ,vector<Point2f> >& fea)
		{
			Mat src_gray;
			cvtColor( fir, src_gray, CV_BGR2GRAY );
			fea.first.resize(kptDet_maxCorners);
			goodFeaturesToTrack( src_gray,
					   fea.first,
					   kptDet_maxCorners,
					   kptDet_qualityLevel,
					   kptDet_minDistance,
					   Mat(),
					   kptDet_blockSize,
					   kptDet_useHarrisDetector,
					   kptDet_k );

			vector<uchar> status;
			vector<float> err;
		
		    calcOpticalFlowPyrLK(fir,sec,
				 fea.first,fea.second,status,err,
				 Size(kptTrack_winsize,kptTrack_winsize),
				 kptTrack_maxlevel, termcrit, 0, 0.001);

			for(size_t ti=0;ti<status.size();++ti)
			{
				if(status[ti]!='\1')
					fea.second[ti]=Point2f(-1.0,-1.0);
			}

		},imgs[i],imgs[i+1],features[i]);
	}

	for (size_t i = 0; i < thr_num1; i++)
	{
		thrds1[i].join();
	}


	size_t thr_num2=imgs.size()-2;

	
	vector<vector<vector<int> > > weights_buffer(corres.size());
	//weights_buffer.resize();

	for(size_t i=0;i<weights_buffer.size();++i)
	{
		weights_buffer[i].resize(features[i].second.size(),vector<int>(features[i+1].first.size()));
	}


	
	thread* thrd2=new thread[thr_num2];

	for (size_t i = 0; i < thr_num2; i++)
	{
		thrd2[i]=thread([](const vector<Point2f>& fir,const vector<Point2f>& sec,vector<vector<int> >& buffer,map<size_t,size_t>& corr){
			for (size_t ti = 0; ti < fir.size(); ti++)
			{
				for (size_t tj = 0; tj < sec.size(); tj++)
				{
					buffer[ti][tj]=10*(abs(fir[ti].x-sec[tj].x)+abs(fir[ti].y-sec[tj].y));
				}
			}


			munkres test;
			test.set_diag(false);
			auto& x=buffer;
			test.load_weights(x);
			int cost;
			bool inverse;

		   int num_rows,num_columns;// = std::min(x.size(), x[0].size()), num_columns = std::max(x.size(), x[0].size());

		   if(fir.size()<sec.size())
		   {
			   inverse=false;
			   num_rows=fir.size();
			   num_columns=sec.size();
		   }
		   else
		   {
			   inverse=true;
			   num_rows=sec.size();
			   num_columns=fir.size();
		   }

			ordered_pair *p = new ordered_pair[num_rows];
			cost = test.assign(p);
	
			//corr.resize(num_rows);
			//output size of the matrix and list of matched vertices
			std::cerr << "The ordered pairs are \n";
			for (int i = 0; i < num_rows; i++)
			{
				if(!inverse)
				{
					corr[p[i].row]=p[i].col;
				}
				else
				{
					corr[p[i].col]=p[i].row;

				}
//				std::cerr << "(" << p[i].row << ", " << p[i].col << ")" << std::endl;
			}
//			std::cerr << "The total cost of this assignment is " << cost << std::endl;
//			std::cerr << "Dimensions: (" << num_rows << ", " << num_columns << ")" << std::endl;


		},features[i].second,features[i+1].first,weights_buffer[i],corres[i]);
	}

	for(size_t i=0;i<thr_num2;i++)
	{
		thrd2[i].join();
	}

	delete[] thrds1;
	delete[] thrd2;

	return pair< vector<pair<vector<Point2f> ,vector<Point2f> > >, vector<map<size_t,size_t>  > >(features,corres);
}


auto incrementalTrajectoryDetect(const vector<Mat>& imgs)-> vector<vector<Point2f> >
{
	assert(imgs.size()>=2);
	
	
	vector<vector<Point2f> > trajs;

	trajs.reserve(100000);

	
	auto sth=incrementalTrajectoryDetectCore(imgs);

	vector<vector<bool> > markers(sth.first.size());
	

	for (size_t i = 0; i < sth.first.size(); i++)
	{
		markers[i].resize(sth.first[i].first.size(),false);
	}

	for (size_t i = 0; i < sth.first.size()-1; i++)
	{
		for (size_t j = 0; j < markers[i].size(); j++)
		{
			if(!markers[i][j])
			{
				vector<Point2f> traj;
				traj.reserve(imgs.size());
				traj.push_back(sth.first[i].first[j]);
				markers[i][j]=true;
				Point2f& cur_p=sth.first[i].second[j];
				int cur_frame=i;
				int cur_index=j;
				while((cur_frame<sth.first.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0)&&(sth.second[cur_frame].count(cur_index)))
				{
					traj.push_back(sth.first[cur_frame+1].first[sth.second[cur_frame][cur_index]]);
					markers[cur_frame+1][sth.second[cur_frame][cur_index]]=true;
					cur_p=sth.first[cur_frame+1].second[sth.second[cur_frame][cur_index]];
					cur_index=sth.second[cur_frame][cur_index];
					++cur_frame;
				}
				
				if(cur_frame==(sth.first.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0))
					traj.push_back(cur_p);

				trajs.push_back(traj);
			}
		}
	}

	return trajs;
}


/*
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
			 calcOpticalFlowPyrLK(imgs[index-i],imgs[index-i-1],
				 cornerss[i],cornerss[i+1],status,err,
				 Size(kptTrack_winsize,kptTrack_winsize),
				 kptTrack_maxlevel, termcrit, 0, 0.001);
		 }

		
#ifdef presentationMode_on
		 drawTrajs(copy,cornerss);


		imshow( "what ever", copy );
#endif
		return cornerss;
		

	

}*/