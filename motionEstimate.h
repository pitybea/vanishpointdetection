#pragma once



vector<double> Img2Serph(double x,double y)
{
	double alpha= -x*2*pi/8000+2*pi;
	double beta=  -pi*y/4000+pi/2;

	vector<double> rslt(3);
	rslt[0]=cos(beta)*cos(alpha);
	rslt[1]=cos(beta)*sin(alpha);
	rslt[2]=sin(beta);
	return rslt;
}


vector<pair<vector<double>,vector<double> > >  inMotionFileSingle(const string& fln)
{
	FILE* fp;
	fopen_s(&fp,fln.c_str(),"r");

	int trj_ind;
	fscanf_s(fp,"%d ",&trj_ind);

	vector<pair<vector<double>,vector<double> > > rslt;

	rslt.reserve(traj::kptDet_maxCorners);
	while(trj_ind!=-1)
	{
		double d1,d2,d3,d4;
		fscanf_s(fp,"%lf %lf %lf %lf\n",&d1,&d2,&d3,&d4);
		pair<vector<double>,vector<double> > one;
		one.first=Img2Serph(d1,d2);
		one.second=Img2Serph(d3,d4);
		rslt.push_back(one);
		fscanf_s(fp,"%d ",&trj_ind);
	}
	

	fclose(fp);
	return rslt;
}


vector<double> estimateMotion(const vector<pair<vector<double>,vector<double> > >& pts)
{

	vector<double> rslt; 
	rslt.resize(3,0.0);

	vector<vector< int >> xyz_ind(3);
	for (int i = 0; i < 3; i++)
	{
		xyz_ind[i].reserve(pts.size());
	}

	for (int i = 0; i < pts.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			if (pts[i].first[j]<dis_lmt)
			{
				xyz_ind[j].push_back(i);
			}
		}
	}
	
	for (int i = 0; i < 3; i++)
	{
		vector<double> diss;
		diss.resize(xyz_ind[i].size());
		vector<int> indx;
		indx.resize(xyz_ind[i].size(),0);
		for (int j = 0; j < xyz_ind[i].size(); j++)
		{
			diss[j]=pts[j].second[i]-pts[j].first[i];
			indx[j]=j;
		}
		FromSmall(diss,diss.size(),indx);

		int st=( perc_lmt-1.0)* diss.size()/2*(-1);
		int ed=diss.size()+( perc_lmt-1.0)* diss.size()/2;
		for (int j = st; j < ed; j++)
		{
			rslt[i]+=diss[indx[j]];
		}
		if(ed>st)
			rslt[i]/=(ed-st);
	}

	return rslt;
};


void EstimateTransofrmationsimple(const vector<string>& flns)
{

	vector<vector<double> > trans(flns.size());

	#pragma omp parallel for
	for (int i = 0; i < flns.size(); i++)
	{
		cout<<flns[i]<<" ";
		auto sth=inMotionFileSingle(flns[i]);
		trans[i]=estimateMotion(sth);
	}

	for (int i = 1; i < trans.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans[i][j]+=trans[i-1][j];
		}
	}
	FILE* fp;
	fopen_s(&fp,"trans.txt","w");
	fprintf(fp,"0.0 0.0 0.0\n");
	for (int i = 0; i < trans.size(); i++)
	{

		fprintf(fp,"%lf %lf %lf\n",trans[i][0],trans[i][1],trans[i][2]);
	}
	fclose(fp);
}

void EstimateTransofrmationsimple(const string& fln)
{
	FILE* fp;
	fopen_s(&fp,fln.c_str(),"r");
	int frm_num;
	fscanf_s(fp,"%d",&frm_num);
	fclose(fp);

	vector<string> flns(frm_num-1);
	for (int i = 0; i < frm_num-1; i++)
	{
		flns[i]=fln+to_string(i)+"_"+to_string(i+1)+".txt";
	}
	vector<vector<double> > trans(flns.size());

	#pragma omp parallel for
	for (int i = 0; i < flns.size(); i++)
	{
		cout<<flns[i]<<" ";
		auto sth=inMotionFileSingle(flns[i]);
		trans[i]=estimateMotion(sth);
	}

	for (int i = 1; i < trans.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans[i][j]+=trans[i-1][j];
		}
	}
	fopen_s(&fp,"trans.txt","w");
	fprintf(fp,"0.0 0.0 0.0\n");
	for (int i = 0; i < trans.size(); i++)
	{

		fprintf(fp,"%lf %lf %lf\n",trans[i][0],trans[i][1],trans[i][2]);
	}
	fclose(fp);
}