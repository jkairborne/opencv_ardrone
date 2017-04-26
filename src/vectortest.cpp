// erasing from vector
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <sstream>

int within_bounds(int index);

int main ()
{
int NUMPTS = 4;	

    float rtn_vec[NUMPTS][NUMPTS];
	for(int i=0; i<4;i++) 
	{
		//We use -1 from the old vector size because the mean is included in old vector
		for(int j = 0; j < (4); j++)
			{
			// Calculate distance from each old point (except the last one, which corresponds to the mean
			rtn_vec[i][j] = rand() % 12 + rand()% 13 + rand() % 15;
			std::cout << rtn_vec[i][j] << "  ";
		}// end of internal nested for
	std::cout << std::endl;
	} // end of external nested for
std::cout <<std::endl;

    float sum_dist_vec[4] = {0,0,0,0};
    int min_dist_ind = 0;

std::cout << sum_dist_vec[0] << " "  << sum_dist_vec[1] << " "  << sum_dist_vec[2] << " "  << sum_dist_vec[3]; 
	// We now have a 4x4 matrix populated by the distances to nearby points.
	// We essentially want to select the lowest sum possible that still incorporates all distances.
    // The code below calculates the various ways of shifting the rectangle
for (int i = 0; i<NUMPTS; i++)
{
    for (int j = 0; j<NUMPTS; j++)
    {        //THIS IS WRONG
        int k = within_bounds(j+i);
        sum_dist_vec[i] += rtn_vec[j][k];
        std::cout << "i = " << i << " j = " << j << "k = " << k << "sum_dist_vec" << sum_dist_vec[i] << std::endl;
    }
    if (sum_dist_vec[i] < sum_dist_vec[min_dist_ind])
    {
        min_dist_ind = i;
        std::cout << std::endl << "found a new min : " << i << std::endl;
    }
}

std::vector<int> pts_to_identify(4);
pts_to_identify[0] = 10;
pts_to_identify[1] = 20;
pts_to_identify[2] = 30;
pts_to_identify[3] = 40;
    std::cout << "pts_to_identify : " <<\
                 pts_to_identify[0] << "   " <<\
                 pts_to_identify[1] << "   " <<\
                 pts_to_identify[2] << "   " <<\
                 pts_to_identify[3] << "   " << std::endl;

std::vector<int> output_vector(4);


for (int p = 0; p < NUMPTS; p++)
{
    //THIS IS ALSO PROBABLY WRONG;
    int new_index = within_bounds(p+min_dist_ind);
    output_vector[new_index] = pts_to_identify[p];
    std::cout << "output vector : " << output_vector[new_index] << std::endl;
}
    std::cout << "output vector : " <<\
                 output_vector[0] << "   " <<\
                 output_vector[1] << "   " <<\
                 output_vector[2] << "   " <<\
                 output_vector[3] << "   " <<
                 std::endl;








  return 0;
}

int within_bounds(int index)
{

// If the index is greater than numpoints, that means that we want to be wrapping around the matrix
	if (index >= 4)
	{
		return (index-4);
	}
	else {return index;}
}



