
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

auto incrementalTrajectoryDetect(vector<vector<Point2f> >& features, vector<map<int,int>>& sth)-> vector<vector<Point2f> >;