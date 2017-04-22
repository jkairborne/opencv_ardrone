// erasing from vector
#include <iostream>
#include <vector>

int main ()
{
std::vector< std::vector<float> > vec(4, std::vector<float>(4));

	for(std::vector<float>::size_type i = 0; i < 4; i++) 
	{
		//We use -1 from the old vector size because the mean is included in old vector
		for(std::vector<float>::size_type j = 0; j < 4; j++)
		{
			vec[i][j] = i*j;
//			std::cout << "In the second callback, i = " << i << " j = " << j << " new element: " << rowvec[j];
		}// end of internal nested for

		// Calculate distance from each old point (except the last one, which corresponds to the mean
	}

  std::cout << "vec contains:";
  for (unsigned i=0; i<4; ++i)
  {
  		for(unsigned j=0; j<4;j++)
  		{
		    std::cout << ' ' << vec[i][j];
    	}

    		  std::cout << '\n';
	}

  return 0;
}
