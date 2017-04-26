// erasing from vector
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <sstream>

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
			rtn_vec[i][j] = rand() % 25;
		}// end of internal nested for
	} // end of external nested for



    float sum_dist_vec[24];
    int min_dist_ind = 0;
	int index = 0;
    int order[NUMPTS];
    
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
							std::cout << "i : " << i << " j : " << j << " k : " << k << " m : " << m;
							sum_dist_vec[index] = rtn_vec[0][i] + rtn_vec[1][j] + rtn_vec[2][k] + rtn_vec[3][m];
							if (sum_dist_vec[index] < sum_dist_vec[min_dist_ind]) 
							{
								min_dist_ind = index;
								order[0] = i;
								order[1] = j;
								order[2] = k;
								order[3] = m;
								std::cout << std::endl<< "Values: " << order[0] << order[1] << order[2] << order[3]<< std::endl;
							}
							std::cout << ", minimum index is: " << min_dist_ind << std::endl;
							index++;
						}
					}
				}
			}
		}//endif 
	}//endfor
}
//std::cout << "Just before declaring test";

std::cout << "Values: " << order[0] << order[1] << order[2] << order[3];
/*
    ROS_INFO("In pvt_ident_6");//2
for (int p = 0; p < 4; p++)
{
    int index2 = order[p];
    std::cout << "In the for loop" << index2;
    output_vector[p] = cv::Point2f(p,0);
    std::cout << "output vector : " << output_vector << std::endl;
    //output_vector[index2].x = pts_to_identify[p].x;
   // output_vector[index2].y = pts_to_identify[p].y;
     std::cout << pts_to_identify[p];
}

int a,b,c,d;

str_rtn >> a;
str_rtn >> b;
str_rtn >> c;
str_rtn >> d;

std::cout << std::endl << "New values: " << '\n' << a << b << c << d;
*/





  return 0;
}
