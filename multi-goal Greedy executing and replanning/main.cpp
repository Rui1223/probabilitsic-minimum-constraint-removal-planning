#include <cstdio>
#include <cassert>
#include <cmath>  // pow(), sqrt()
#include <iostream> 
#include <fstream> // stream class to write on files
#include <cstdio> // printf() 
#include <vector> // std::vector
#include <algorithm> // std::set_union, std::sort, std::find
#include <bitset> // for bitwise operation
#include <map> // std::map
#include <functional>
#include <set>
#include <cstdlib> // for std::srand()
#include <string> // std::string, std::to_string
#include <queue>
#include <deque>
#include <random>
#include <limits>

class test_t
{
	int m_h;
	int m_g;
	int m_f;

public:
	test_t(int h, int g);
	~test_t();
	int geth() { return m_h; }
	int getf() { return m_f; }
	void setf(int f) { m_f = f; }
	void printf() {std::cout << m_f << "\n";}
};

test_t::test_t(int h, int g)
{
	m_h = h;
	m_g = g;
}

test_t::~test_t()
{
	std::cout << "Destructor for test_t object " 
			<< m_h << " " << m_g << " is called.\n";
}



int main()
{
	double tw = 0.2;
	std::cout << tw * log(tw) << "\n";


	return 0;
}