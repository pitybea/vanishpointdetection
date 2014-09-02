
#pragma once 
#include "../FileIO/FileInOut.h"
//#include <cv.h>
#include "utils.h"
#include "TrajSettings.inl"

#include "parallelSetting.inl"
//using namespace vanish;
using namespace cv; //quick and dirty
#include <vector>
#include <tuple>

auto incrementalTrajectoryDetect(const vector<string>& imgs)-> vector<vector<Point2f> >;

void removeStaticTrajectories(vector<vector<Point2f> > &trajs);


auto effectTraj(const vector<Point2f>& inp)->vector<Point2f>;



void simpleTrackPrint(const vector<string>& flnms, const string& flnm);

void saveTraj_BannoFormat( char* oup,const vector<string>& flnms,const vector<vector<Point2f> >& trajs,const Mat & img);


void saveTraj_parallel(const string& inpf,const vector<vector<Point2f> >& trajs);


void patchDealSave(const vector<vector<string> >& fileNames,const vector<vector<int> >& indxs,const char*  lstFileName,const char*  LogFileName,const char*);

template<class Pnt>
auto incrementalTrajectoryDetect(vector<vector<Pnt> >& features, vector<map<int,int>>& sth)-> pair<vector<pair<int,int> >,  vector<vector<Pnt> > >
	{
	//assert(imgs.size()>=2);
	
	
	vector<pair<int,int> > sttend;

	vector<vector<Pnt> > trajs;




	vector<vector<bool> > markers(features.size());
	

	for (size_t i = 0; i < features.size(); i++)
	{
		markers[i].resize(features[i].size(),false);
	}

	for (size_t i = 0; i < features.size()-1; i++)
	{
		cout<<"processing frame " <<i<<endl;
		for (size_t j = 0; j < markers[i].size(); j++)
		{
			if(!markers[i][j])
			{
				pair<int,int> stend;
				vector<Pnt> traj;
		
				stend.first=i;

				traj.push_back(features[i][j]);
				markers[i][j]=true;
				


				int cur_frame=i;
				int cur_index=j;

				
				while( (cur_frame<features.size()-1) && (sth[cur_frame].count(cur_index)))
				{
					int indd=sth[cur_frame][cur_index];
				
					traj.push_back(features[cur_frame+1][indd]);

					markers[cur_frame+1][indd]=true;
				
					cur_index=indd;
					++cur_frame;			
				}
				stend.second=cur_frame;
				if(stend.second>stend.first)
				{
				trajs.push_back(traj);
				sttend.push_back(stend);}
			}
		}
	}
	cout<<"feature tracing finished"<<endl;
	return make_pair(sttend,trajs);
}