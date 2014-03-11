#include <stdio.h>
#include <stdlib.h>
#include "../dnn/rbm.h"
#include "../dnn/mnist.h"
#include <Windows.h>
#include <direct.h>
#include <algorithm>
using namespace std;

string flnms[4]=
{

	"train-images-idx3-ubyte.gz",
	"train-labels-idx1-ubyte.gz",
	"t10k-images-idx3-ubyte.gz",
	"t10k-labels-idx1-ubyte.gz"
};

char* argv[]={"","train","train-images-idx3-ubyte.gz",
	"train-labels-idx1-ubyte.gz"};
int argc=4;
int main_()
{
	if (argc != 4) {
		std::cerr << "Usage: " << argv[0] << "<train-simple|train|test> <image-file> <label-file>" << std::endl;
		return -1;
	}
	_chdir("E:\\vanish\\VanishPntDtction\\dnn\\mnist");
	std::vector<Sample> samples;
	int n = mnist::read(argv[2], argv[3], samples);
	if (n <= 0) {
		std::cerr << "failed to read mnist data files: " << argv[2] << " ," << argv[3] << std::endl;
		return -1;
	}

	std::string command = argv[1];

	// initialize pallet


	// initialize data
	int data_size = samples[0].data_.size();
	std::vector<Vector> inputs(n);
	std::vector<Vector> targets(n);
	for (size_t i=0; i< n; ++i) {
		const Sample& sample = samples[i];
		Vector& input = inputs[i];
		Vector& target = targets[i];

		input.resize(data_size); target.resize(10);
		for (size_t j=0; j<data_size; ++j) input[j] = sample.data_[j] / 255.0; // > 30 ? 1.0: 0.0; // binary 
		target[sample.label_] = 1.0;
	}

	// progress monitoring
	auto progress = [](DeepBeliefNet& dbn) {
		static int i = 0;
		std::string name = "rbm-" + std::to_string(i++);
#if 0
		int width = 0, height = 0;
		Vector pixels;
		dbn.to_image(pixels, width, height);

		Magick::Image img(Magick::Geometry(width * 2, height * 2), Magick::Color(255,255,255));
		for (size_t x=0; x < width * 2; ++x) {
				for (size_t y=0; y < height * 2; ++y) {
					int i =  int(abs(pixels[int(y / 2 * width + x / 2)] * 255));
					if (i > 255 || i < 0) i = 255;
					img.pixelColor(x, y, pallet[i]);
				}
		}
		std::string fn = name + ".png";
		img.write(fn.c_str());
#endif

		std::ofstream f(name + ".dat", std::ofstream::binary);
		dbn.store(f);
	};
	
	// training and testing functions
	auto train_dbn_simple = [&]() {
		DeepBeliefNet dbn;
		std::vector<int> vint1;
		vint1.push_back(data_size);
		vint1.push_back(300);
		vint1.push_back(300);
		vint1.push_back(500);
		vector<int> vint2;
		vint2.push_back(0);vint2.push_back(0);vint2.push_back(10);
		dbn.build(vint1, vint2);

		LRBM::Conf conf;
		conf.max_epoch_ = 6; conf.max_batches_ = 100; conf.batch_size_ = 100;
		
		dbn.train(inputs, targets, dbn.max_layer(), conf, progress);
	
		std::ofstream f("dbn-s.dat", std::ofstream::binary);
		dbn.store(f);
	};
	
	auto train_dbn = [&]() {
		DeepBeliefNet dbn;
		vector<int> vint;
		vint.push_back(data_size);
		vint.push_back(300);
		vint.push_back(300);
		vint.push_back(500);
		vint.push_back(10);
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
			conf.max_epoch_ = 2; conf.max_batches_ = 300; conf.batch_size_ = 200;
		}
		else {
			conf.max_epoch_ = 10; conf.max_batches_ = 300; conf.batch_size_ = 200;
			dbn.pretrain(inputs, conf, progress);
		}

		conf.max_epoch_ = 10; conf.max_batches_ /= 5; conf.batch_size_ *= 5;
		dbn.backprop(inputs, targets, conf, progress);

		std::ofstream f("dbn.dat", std::ofstream::binary);
		dbn.store(f);
	};

	auto train_autoencoder = [&]() {
		AutoEncoder enc;
		vector<int> vint;
		vint.push_back(data_size);
		vint.push_back(500);
		vint.push_back(30);
		vint.push_back(500);
		vint.push_back(data_size);
		enc.build(vint);

		auto& rbm = enc.rbms_[enc.max_layer() / 2 - 1];
		rbm->type_ = RBM::Type::LINEAR;

		LRBM::Conf conf;
		conf.max_epoch_ = 10; conf.max_batches_ = 50; conf.batch_size_ = 100;
		enc.pretrain(inputs, conf, progress);

		conf.max_epoch_ = 10; conf.max_batches_ /= 5; conf.batch_size_ *= 5;
		enc.backprop(inputs, conf, progress);

		std::ofstream f("enc.dat", std::ofstream::binary);
		enc.store(f);
	};

	auto test_dbn = [&](bool is_simple) {
		DeepBeliefNet rbm;
		std::string file = is_simple? "dbn-s.dat" : "dbn.dat";
		std::ifstream f(file, std::ifstream::binary);
		rbm.load(f);

		size_t correct = 0, second = 0;
		for (size_t i = 0; i < samples.size(); ++i) {
			const Sample& sample = samples[i];

			std::vector<int> idx(10);
			for(int i=0; i<10; ++i) idx[i] = i;

			static Vector nil;
			Vector output(10);
			if (is_simple)
				rbm.predict(inputs[i], output, nil);
			else
				rbm.predict(inputs[i], nil, output);

			std::sort(idx.begin(), idx.end(), [&output](int x, int y) { return output[x] > output[y]; });

			if (idx[0] == (int)sample.label_) ++ correct;
			else if (idx[1] == (int)sample.label_) ++ second;


			if ((i + 1) % 100 == 0)	std::cout << "# " << correct << "/" << i + 1 << " recognized. 1st: " << (correct * 100.0/ (i+1)) << "%, 1st+2nd: " << (correct + second) * 100.0/(i+1) << "%" << std::endl;
		}

		std::cout << "# " << correct << " recognized." << std::endl;
	};

	// execute commands
	if (command == "train") train_dbn();
	else if (command == "train-simple") train_dbn_simple();
	else if (command == "test") test_dbn(false);
	else if (command == "test-simple") test_dbn(true);
	else if (command == "train-encoder") train_autoencoder();
	else {
		std::cerr << "unrecognized command: " << command << std::endl;	
	}

	return 0;
}