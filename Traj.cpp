#include "Traj.h"
#include <thread>
#include <map>
#include "munkres.h"
using namespace traj;




auto incrementalTrajectoryDetectCore(const vector<string>& imgs)-> pair<vector< pair<vector<Point2f> ,vector<Point2f> > >, vector<map<size_t,size_t>>>
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

	//thread* thrds1=new thread[thr_num1];

	
	#pragma omp parallel for
	for (int i = 0; i < thr_num1; i++)
	{
		const string& _fir=imgs[i];
		const string& _sec=imgs[i+1];
		pair<vector<Point2f> ,vector<Point2f> >& fea =features[i];
		int index=i;
	
		Mat fir=imread(_fir);
		Mat sec=imread(_sec);
//		pair<vector<Point2f> ,vector<Point2f> >& fea=;
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
		TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, kptTrack_iter, kptTrack_epsin);
		calcOpticalFlowPyrLK(fir,sec,
				fea.first,fea.second,status,err,
				Size(kptTrack_winsize,kptTrack_winsize),
				kptTrack_maxlevel, termcrit, 0, 0.001);

		for(size_t ti=0;ti<status.size();++ti)
		{
			if(status[ti]!='\1')
				fea.second[ti]=Point2f(-1.0,-1.0);
		}
		cout<<"finished frame "<<index<<"\t"<<endl;
	
	}

	

	size_t thr_num2=imgs.size()-2;

	
	cout<<"feature tracking finished"<<endl;

	
	//thread* thrd2=new thread[thr_num2];
	#pragma omp parallel for
	for (int i = 0; i < thr_num2; i++)
	{

		//features[i].second,features[i+1].first,ref(corres[i]
		const vector<Point2f>& fir=features[i].second;
		const vector<Point2f>& sec=features[i+1].first;
		map<size_t,size_t>& corr=corres[i];
			
		vector<bool> used(sec.size(),false);

		for (size_t ti = 0; ti < fir.size(); ti++)
		{
			vector<double> dis;
			dis.reserve(40);
			vector<int> indx;
			indx.reserve(40);
			for (size_t tj = 0; tj < sec.size(); tj++)
			{
				double tdis=abs(fir[ti].x-sec[tj].x)+abs(fir[ti].y-sec[tj].y);
				if(tdis<pnt_dis_threshold)
				{
					dis.push_back(tdis);
					indx.push_back(tj);
				}
			}
			if(dis.size()>0)
			{
				FromSmall(dis,dis.size(),indx);
				for (size_t tk = 0; tk < dis.size(); tk++)
				{
					if(!used[indx[tk]])
					{
						corr[ti]=indx[tk];
						used[indx[tk]]=true;
						break;
					}
				}
			}
		}


	}


	cout<<"feature corresponding finished"<<endl;


	return pair< vector<pair<vector<Point2f> ,vector<Point2f> > >, vector<map<size_t,size_t>  > >(features,corres);
}


auto incrementalTrajectoryDetect(const vector<string>& imgs)-> vector<vector<Point2f> >
{
	assert(imgs.size()>=2);
	
	
	vector<vector<Point2f> > trajs;

	trajs.reserve(1000000);

	
	auto sth=incrementalTrajectoryDetectCore(imgs);

	vector<vector<bool> > markers(sth.first.size());
	

	for (size_t i = 0; i < sth.first.size(); i++)
	{
		markers[i].resize(sth.first[i].first.size(),false);
	}

	for (size_t i = 0; i < sth.first.size()-1; i++)
	{
		cout<<"processing frame " <<i<<endl;
		for (size_t j = 0; j < markers[i].size(); j++)
		{
			if(!markers[i][j])
			{
				vector<Point2f> traj;
				traj.reserve(imgs.size());
				if(i!=0)
					for (int k = 0; k < i; ++k)
					{
						traj.push_back(Point2f(0.0,0.0));
					}
				traj.push_back(sth.first[i].first[j]);
				markers[i][j]=true;
				Point2f& cur_p=sth.first[i].second[j];
				int cur_frame=i;
				int cur_index=j;
				int effec_num=1;
				while((cur_frame<sth.first.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0)&&(sth.second[cur_frame].count(cur_index)))
				{
					traj.push_back(sth.first[cur_frame+1].first[sth.second[cur_frame][cur_index]]);
					markers[cur_frame+1][sth.second[cur_frame][cur_index]]=true;
					cur_p=sth.first[cur_frame+1].second[sth.second[cur_frame][cur_index]];
					cur_index=sth.second[cur_frame][cur_index];
					++cur_frame;
					++effec_num;
				}
				
				if(cur_frame==(sth.first.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0))
					traj.push_back(cur_p);
				if(effec_num>1)
				while (traj.size()<imgs.size())
				{
					traj.push_back(Point2f(0.0,0.0));
				}

				if(effec_num>1)
					trajs.push_back(traj);
			}
		}
	}
	cout<<"feature tracing finished"<<endl;
	return trajs;
}

auto effectTraj(const vector<Point2f>& inp)->vector<Point2f>
{
	vector<Point2f> rslt;
	rslt.reserve(inp.size());

	int in=0;
	while((in<inp.size())&&(inp[in].x<0.01)&&(inp[in].y<0.01))
		++in;
	while((in<inp.size())&&(inp[in].x>0.01)&&(inp[in].y>0.01))
	{	rslt.push_back(inp[in]);
		++in;
	}
	return rslt;
}

void removeStaticTrajectories(vector<vector<Point2f> > &trajs)
{
	cout<<"remove static trjs"<<endl;
	vector<bool> static_markers(trajs.size(),false);
#pragma omp parallel for

	for(int i=0;i<trajs.size();i++)
	{
		auto etraj=effectTraj(trajs[i]);
		
		double dis=0.0;
		if(etraj.size()>1)
		{
			for (int j = 0; j < etraj.size()-1; j++)
			{
				double tdis=abs(etraj[j+1].x-etraj[j].x)+abs(etraj[j+1].y-etraj[j].y);
				dis+=tdis;
			}
			if((dis/etraj.size())<static_pnt_cre/2)
				static_markers[i]=true;
		}
	}

	auto sth=trajs.begin();
	for (int i = trajs.size()-1;i>=0;--i)
	{
		if(static_markers[i])
			trajs.erase(sth+i);
	}

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