// erasing from vector
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <sstream>

int main ()
{
int NUMPTS = 4;	
	 
	std::vector<float> sum_dist_vec(24);
	int min_dist_ind = 0;
	int index = 0;
	int order[4];
	std::stringstream str_rtn;
	
	
for (int i = 0; i < NUMPTS; i++)
{
	for(int j = 0; j < NUMPTS; j++)
	{
		if (i!=j)
		{
			for(int k=0; k <NUMPTS; k++)
			{
				if (i!=k && j!=k)
				{
					for (int m=0; m<NUMPTS; m++)
					{
						if (i!=m && j!=m && k!=m)
						{
							std::cout << "i : " << i << " j : " << j << " k : " << k << " m : " << m << std::endl;
							sum_dist_vec[index] = rand()%100;
							if (sum_dist_vec[index] < sum_dist_vec[min_dist_ind]) 
							{
								min_dist_ind = index;
								order[0] = i;
								order[1] = j;
								order[2] = k;
								order[3] = m;
								//Empty previous string stream
								str_rtn.str("");
								str_rtn << i << " " << j <<" " << k <<" " << m;
								std::cout << str_rtn.str() << std::endl;
							}
							//std::cout << "minimum index is: " << min_dist_ind;
							index++;
						}
					}
				}
			}
		}//endif 
	}//endfor
}

int a,b,c,d;

str_rtn >> a;
str_rtn >> b;
str_rtn >> c;
str_rtn >> d;

std::cout << std::endl << "New values: " << '\n' << a << b << c << d;






  return 0;
}
