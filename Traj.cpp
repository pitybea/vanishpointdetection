#include "Traj.h"
#include <thread>
#include <map>
#include "munkres.h"
using namespace traj;

using namespace parallel;


auto incrementalTrajectoryDetectCore(const vector<string>& imgs,vector<pair<vector<Point2f> ,vector<Point2f> > >& features)-> vector<map<size_t,size_t>>
{
	assert(imgs.size()>=2);

	
	
	vector<map<size_t,size_t> > corres(imgs.size()-2);
	
	


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
#ifdef _DEBUG
		cout<<"finished frame "<<index<<"\t"<<endl;
#endif
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


	return corres;
}


auto incrementalTrajectoryDetect(const vector<string>& imgs)-> vector<vector<Point2f> >
{
	assert(imgs.size()>=2);
	
	
	vector<vector<Point2f> > trajs;

	trajs.reserve(1000000);

	vector<pair<vector<Point2f> ,vector<Point2f> > > features;
	features.resize(imgs.size()-1);
	
	auto sth=incrementalTrajectoryDetectCore(imgs,features);

	vector<vector<bool> > markers(features.size());
	

	for (size_t i = 0; i < features.size(); i++)
	{
		markers[i].resize(features[i].first.size(),false);
	}

	for (size_t i = 0; i < features.size()-1; i++)
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
				traj.push_back(features[i].first[j]);
				markers[i][j]=true;
				Point2f& cur_p=features[i].second[j];
				int cur_frame=i;
				int cur_index=j;
				int effec_num=1;
				while((cur_frame<features.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0)&&(sth[cur_frame].count(cur_index)))
				{
					traj.push_back(features[cur_frame+1].first[sth[cur_frame][cur_index]]);
					markers[cur_frame+1][sth[cur_frame][cur_index]]=true;
					cur_p=features[cur_frame+1].second[sth[cur_frame][cur_index]];
					cur_index=sth[cur_frame][cur_index];
					++cur_frame;
					++effec_num;
				}
				
				if(cur_frame==(features.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0))
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


auto incrementalTrajectoryDetect_Effect(const vector<string>& imgs,vector<pair<vector<Point2f> ,vector<Point2f> > >& features)-> pair< vector<vector<Point2f> >, vector<pair<int,int> > >
{
	assert(imgs.size()>=2);
	
	pair< vector<vector<Point2f> >, vector<pair<int,int> > > rslt;


	vector<vector<Point2f> >& trajs=rslt.first;

	trajs.reserve(patch_size* kptDet_maxCorners/4 );

	vector<pair<int,int> >& startEndRecords=rslt.second;

	startEndRecords.reserve(patch_size* kptDet_maxCorners/4 );

	
	auto sth=incrementalTrajectoryDetectCore(imgs,features);

	vector<vector<bool> > markers(features.size());
	

	for (size_t i = 0; i < features.size(); i++)
	{
		markers[i].resize(features[i].first.size(),false);
	}

	for (size_t i = 0; i < features.size()-1; i++)
	{
		cout<<"processing frame " <<i<<endl;
		for (size_t j = 0; j < markers[i].size(); j++)
		{
			if(!markers[i][j])
			{
				vector<Point2f> traj;
				pair<int,int> sttNd;
				traj.reserve(imgs.size());
				sttNd.first=i;
//				if(i!=0)
//					for (int k = 0; k < i; ++k)
//					{
//						traj.push_back(Point2f(0.0,0.0));
//					}

				traj.push_back(features[i].first[j]);
				markers[i][j]=true;
				Point2f& cur_p=features[i].second[j];
				int cur_frame=i;
				int cur_index=j;
				int effec_num=1;
				while((cur_frame<features.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0)&&(sth[cur_frame].count(cur_index)))
				{
					traj.push_back(features[cur_frame+1].first[sth[cur_frame][cur_index]]);
					markers[cur_frame+1][sth[cur_frame][cur_index]]=true;
					cur_p=features[cur_frame+1].second[sth[cur_frame][cur_index]];
					cur_index=sth[cur_frame][cur_index];
					++cur_frame;
					++effec_num;
				}
				
				if(cur_frame==(features.size()-1)&&(cur_p.x>=0)&&(cur_p.y>=0))
					traj.push_back(cur_p);

				if(effec_num>1)
					sttNd.second=sttNd.first+traj.size();
//				while (traj.size()<imgs.size())
//				{
//					traj.push_back(Point2f(0.0,0.0));
//				}

				if(effec_num>1)
				{
					trajs.push_back(traj);
					startEndRecords.push_back(sttNd);
				}
			}
		}
	}
	cout<<"feature tracing finished"<<endl;

	return rslt;
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


vector<bool> markStaticTrajectories(const vector<vector<Point2f> > &trajs)
{
	cout<<"mark static trjs"<<endl;
	vector<bool> static_markers(trajs.size(),false);
#pragma omp parallel for

	for(int i=0;i<trajs.size();i++)
	{
		auto& etraj=trajs[i];
		
		double dis=0.0;
		for (int j = 0; j < etraj.size()-1; j++)
		{
			double tdis=abs(etraj[j+1].x-etraj[j].x)+abs(etraj[j+1].y-etraj[j].y);
			dis+=tdis;
		}
		if((dis/etraj.size())<static_pnt_cre/2)
			static_markers[i]=true;
		
	}

	
	return static_markers;

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




bool nonEmpty(const Point2f pt)
{
	if((pt.x<0.01)&&(pt.y<0.01))
		return false;

	return true;
}


void saveTraj_BannoFormat(const char* &oup,const vector<string>& flnms,const vector<vector<Point2f> >& trajs,const Mat & img)
{
	FILE* fp;
	fopen_s(&fp,oup,"w");
	fprintf(fp,"%d %d %d %d\n",2* flnms.size(),trajs.size(),img.cols,img.rows);
	for (int i = 0; i < trajs[0].size(); i++)
	{
		for (int j = 0; j < trajs.size(); j++)
		{
			fprintf(fp,"%lf ",(double)trajs[j][i].x);
		}
		fprintf(fp,"\n");
	}

	for (int i = 0; i < trajs[0].size(); i++)
	{
		for (int j = 0; j < trajs.size(); j++)
		{
			fprintf(fp,"%lf ",(double)trajs[j][i].y);
		}
		fprintf(fp,"\n");
	}

	fclose(fp);
}

void saveTraj_parallel(const string& inpf,const vector<vector<Point2f> >& trajs)
{
	#pragma omp parallel for
	for (int i = 0; i < trajs[0].size()-1; i++)
	{
		string fn=inpf+to_string(i)+"_"+to_string(i+1)+".txt";

		FILE* fp;
		fopen_s(&fp,fn.c_str(),"w");
		for (int j = 0; j < trajs.size(); j++)
		{
			if (nonEmpty(trajs[j][i])&&nonEmpty(trajs[j][i+1]))
			{
				fprintf(fp,"%d %lf %lf %lf %lf\n",j,trajs[j][i].x,trajs[j][i].y,trajs[j][i+1].x,trajs[j][i+1].y);
			}
		}
		fprintf(fp,"-1");
		fclose(fp);
	}
}

bool inrange(int i,pair<int,int> rng)
{
	return (i>=rng.first)&&(i<rng.second);
}

void saveTraj_parallel(const string& lstName,const vector<vector<Point2f> >& trajs,const vector<bool>& static_marks,const vector<pair<int,int> >&  statEndIndx,const vector<int> & trajIndxs,int localPatchSize,const vector<int>& lstPositions)
{

	#pragma omp parallel for
	for (int i = 0; i < localPatchSize-1; i++)
	{
		string fn=lstName+to_string( lstPositions[i])+"_"+to_string( lstPositions[i+1])+".txt";

	

		FILE* fp;
		fopen_s(&fp,fn.c_str(),"w");
		for (int j = 0; j < trajs.size(); j++)
		{
			if((!static_marks[j])&&(inrange(i,statEndIndx[j]) )&&(inrange(i+1,statEndIndx[j]) ))

				fprintf(fp,"%d %lf %lf %lf %lf\n",trajIndxs[j],trajs[j][i-statEndIndx[j].first].x,trajs[j][i-statEndIndx[j].first].y,trajs[j][i+1-statEndIndx[j].first].x,trajs[j][i+1-statEndIndx[j].first].y);

		}
		fprintf(fp,"-1");
		fclose(fp);
	}
}

void loadIndLastPts(const char* lstFileName,int indx, vector<int>& okIndex,vector<Point2f>& lastPts )
{
	string fn=lstFileName+to_string( indx-1)+"_"+to_string( indx)+".txt";
	FILE* fp=fopen(fn.c_str(),"r");
	if(fp!=NULL)
	{
		int trj_ind;
		fscanf(fp,"%d ",&trj_ind);
		while(trj_ind!=-1)
		{
			double d1,d2,d3,d4;
			fscanf_s(fp,"%lf %lf %lf %lf\n",&d1,&d2,&d3,&d4);
			okIndex.push_back(trj_ind);
			lastPts.push_back(Point2f(d3,d4));

			fscanf_s(fp,"%d ",&trj_ind);
		}

		fclose(fp);
	}

}

void setIndexsByMatching(vector<Point2f>& lastPts,const vector<vector<Point2f> >& trajs,const vector<pair<int,int> >&  statEndIndx,vector<int> & useIndx,vector<int>& okindx,int localPatchSize,int& currentMaxIndx)
{
	vector<Point2f> sttPts;
	sttPts.reserve(trajs.size());

	

	for (int i = 0; i < trajs.size(); i++)
	{
		if(statEndIndx[i].first==0)
			sttPts.push_back(trajs[i][0]);
		
	}

	const vector<Point2f>& fir=lastPts;
	const vector<Point2f>& sec=sttPts;

	map<int,int> corr;
			
	vector<bool> used(sec.size(),false);

	for (int ti = 0; ti < fir.size(); ti++)
	{
		vector<double> dis;
		dis.reserve(40);
		vector<int> indx;
		indx.reserve(40);
		for (int tj = 0; tj < sec.size(); tj++)
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
			for (int tk = 0; tk < dis.size(); tk++)
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

	useIndx.clear();
	useIndx.resize(trajs.size(),-1);

	for (int i = 0; i < fir.size(); i++)
	{
		if(corr.count(i)!=0)
			useIndx[corr[i]]=okindx[i];
	}
	for (int i = 0; i < useIndx.size(); i++)
	{
		if (useIndx[i]<0)
		{
			useIndx[i]=currentMaxIndx;
			currentMaxIndx+=1;
		}
	}


	lastPts.clear();
	lastPts.reserve(trajs.size());

	okindx.clear();
	okindx.reserve(trajs.size());

	for (int i = 0; i < trajs.size(); i++)
	{



		if(statEndIndx[i].second==localPatchSize)
		{
			lastPts.push_back(trajs[i][trajs[i].size()-1]);
			okindx.push_back(useIndx[i]);
		}
	}


}

void patchDealSave(const vector<vector<string> >& fileNames,const vector<vector<int> >& indxs,const char*  lstFileName,const char*  LogFileName, const char* maxIndFile)
{

	int current_maxIndx=0;
	FILE* ft=fopen(maxIndFile,"r");
	if(ft!=NULL){
		fscanf(ft,"%d",&current_maxIndx);
		fclose(ft);
	}
	auto saveCurrentMaxIndx=[&]()
	{
		FILE* ff=fopen(maxIndFile,"w");
		fprintf(ff,"%d\n",current_maxIndx);
		fclose(ff);
	};
	FILE* fp=	fopen(LogFileName,"a");

	vector<int> index;
	vector<int> okIndx;
	vector<Point2f> lastPts;


	vector<pair<vector<Point2f> ,vector<Point2f> > > features(fileNames[0].size()-1);

	for (size_t i=0;i<fileNames[0].size()-1;++i)
	{
		features[i].first.reserve(kptDet_maxCorners);
		features[i].second.reserve(kptDet_maxCorners);
	}
	


	for (int i = 0; i < fileNames.size(); i++)
	{

		if(fileNames[i].size()<fileNames[0].size())
			features.resize(fileNames[i].size());

		auto tst=incrementalTrajectoryDetect_Effect(fileNames[i],features);


		
		
		
		

		if(i==0)
		{
			loadIndLastPts(lstFileName,indxs[0][0],okIndx,lastPts);
		
		
		}


		setIndexsByMatching(lastPts,tst.first,tst.second,index,okIndx,fileNames[i].size(),current_maxIndx);

		

		auto sttMks=markStaticTrajectories(tst.first);

		saveTraj_parallel(lstFileName,tst.first,sttMks,tst.second,index,fileNames[i].size(),indxs[i]);

		for (int j = 0; j < fileNames[i].size(); j++)
		{
			fprintf(fp,"%s\n",fileNames[i][j].c_str());
		}
		saveCurrentMaxIndx();
	}

	fclose(fp);

}