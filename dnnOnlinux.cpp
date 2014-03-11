#include "../dnn/rbm.h"

#include <vector>
#include <Windows.h>
#include <direct.h>
#include <string>
#include <algorithm>
using namespace std;



void intputData(vector<Vector>& inputs,
	vector<Vector>& targets,string ss1,string ss2)
{
	
	FILE* fp;
	fopen_s(&fp,ss1.c_str(),"r");
	int size,s2;

	fscanf_s(fp,"%d %d\n",&size,&s2);
	inputs.resize(size,vector<float>(s2,0));
	for (int i=0;i<size;i++)
	{
		for (int j=0;j<s2;j++)
		{
			double tem;
			fscanf_s(fp,"%lf ",&tem);
			inputs[i][j]=tem;
		}
		fscanf_s(fp,"\n");
	}

	fclose(fp);
	fopen_s(&fp,ss2.c_str(),"r");
//	int size;
	fscanf_s(fp,"%d\n",&size);
	//result.resize(size,0);
	targets.resize(size,vector<float>(3,0.0));
	for (int i=0;i<size;i++)
	{
		int tem;
		fscanf_s(fp,"%d\n",&tem);
		targets[i][tem]=1.0;
	}

	fclose(fp);


}
void train()
{
	vector<Vector> inputs;
	vector<Vector> targets;
	intputData(inputs,targets,"alltraindata","alltrainlbs");
	cout<<inputs[inputs.size()-1][inputs[0].size()-1];
	auto progress = [](DeepBeliefNet& dbn) 
	{
		static int i = 0;
		string name = "rbm-" + std::to_string(i++);
		ofstream f(name + ".dat", std::ofstream::binary);
		dbn.store(f);
	};
	int data_size=inputs[0].size();
		DeepBeliefNet dbn;
		vector<int> vint;
		vint.push_back(data_size);
		vint.push_back(1500);
		vint.push_back(700);
		vint.push_back(700);
		vint.push_back(300);
		vint.push_back(80);
		vint.push_back(3);
		dbn.build(vint);
		auto& rbm = dbn.output_layer();
		rbm->type_ = RBM::Type::EXP;

  	std::default_random_engine eng(::time(NULL));
  	std::normal_distribution<double> rng(0.0, 1.0);

		LRBM::Conf conf;

		bool resume = false;
		if (resume) {
			std::ifstream f("dbn.dat", std::ifstream::binary);
			dbn.load(f);
			conf.max_epoch_ = 2; conf.max_batches_ = 200; conf.batch_size_ = 150;
		}
		else {
			conf.max_epoch_ = 10; conf.max_batches_ = 200; conf.batch_size_ = 150;
			dbn.pretrain(inputs, conf, progress);
		}

		conf.max_epoch_ = 10; conf.max_batches_ /= 5; conf.batch_size_ *= 5;
		dbn.backprop(inputs, targets, conf, progress);

		std::ofstream f("dbn.dat", std::ofstream::binary);
		dbn.store(f);
	

}
void test()
{
	vector<Vector> inputs;
	vector<Vector> targets;
	vector<int> lbs;
	intputData(inputs,targets,"alltestdata","alltestlbs");
	lbs.resize(inputs.size());
	DeepBeliefNet rbm;
	std::string file = "dbn.dat";
	std::ifstream f(file, std::ifstream::binary);
	rbm.load(f);

		size_t correct = 0, second = 0;
		for (size_t i = 0; i < inputs.size(); ++i) {
			auto& sample = inputs[i];

			std::vector<int> idx(3);
			for(int i=0; i<3; ++i) idx[i] = i;

			static Vector nil;
			Vector output(10);
			rbm.predict(inputs[i], nil, output);

			std::sort(idx.begin(), idx.end(), [&output](int x, int y) { return output[x] > output[y]; });
			lbs[i]=idx[0];
		}
		FILE* fp;fopen_s(&fp,"predict","w");
		fprintf(fp,"%d\n",lbs.size());
		for (int i = 0; i < lbs.size(); i++)
		{
			fprintf(fp,"%d\n",lbs[i]);
		}
		fclose(fp);
		
		std::cout << "# " << correct << " recognized." << std::endl;

}
int _main_()
{
	_chdir("E:\\vanish\\dataset");
	train();
	//int data_size = samples[0].data_.size();
	
}