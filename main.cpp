#include "Timer.hpp"

int main()
{
	Timer t;

	t.reset();
	// loop1
	for (int i=0; i < 10; i++)
	{
		std::cout << i << "\t";
	}
	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";

	// loop2
	for (int i=10; i < 30; i++)
	{
		std::cout << i << "\t";
	}

	std::cout << "\nTimer elapsed: " << t.elapsed() << " seconds\n";

	return 0;
}