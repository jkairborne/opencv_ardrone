// erasing from vector
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <sstream>

void print_vector(std::vector<int> & vec);

int main ()
{
int vecsize = 4;
std::vector<int> tst;
for(int i = 0; i < vecsize; ++i) tst.push_back(i*2+1);
print_vector(tst);
tst.erase(tst.begin() + vecsize -1);
print_vector(tst);
tst.push_back(8);

print_vector(tst);


return 0;
}

void (print_vector(std::vector<int> & vec))
{
	for (int i =0;i<vec.size();i++)
	{
		std::cout << vec[i] << " ";
	}
	std::cout << '\n';
}


