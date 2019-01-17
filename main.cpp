// test how to generate several number with large variance

// normal distribution
#include <iostream>
#include <cstring>
#include <random>
#include <vector>
#include <cstdlib> // for std::srand()
#include <ctime>

void operator/=(std::vector<double> &v, double d)
{
	for (auto &e : v)
	{
		e /= d;
	}
}

// int main(int argc, char** argv)
// {
// 	//std::srand(std::time(0));
// 	std::random_device rd{};
// 	std::mt19937 gen{rd()};

// 	std::vector<double> labelWeights;
// 	double r = 0.0; 

// 	std::normal_distribution<> d{5.0,5.0};
// 	//std::default_random_engine generator;
// 	//std::normal_distribution<double> distribution(20.0,15.0);


// 	for (int kk = 0; kk < 5; kk++)
// 	{
// 		double temp = d(gen);
// 		while (temp <= 0) { temp = d(gen); }
// 		labelWeights.push_back(temp);
// 		r += temp;
// 	}
// 	// normalize
// 	labelWeights /= r;

// 	// print 
// 	for (auto const &w : labelWeights)
// 	{
// 		std::cout << w << "\n";
// 	}

// 	return 0;
// }

int main()
{
	int t1 = 3;
	int t2 = 2;
	std::cout << t1 / t2 << std::endl;
	return 0; 
}