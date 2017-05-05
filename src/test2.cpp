#include <iostream>
#include <Eigen/Dense>
#include "ibvs.h"
using Eigen::MatrixXd;

#include <vector>

int main()
{
   Eigen::MatrixXf m = Eigen::MatrixXf::Random(8, 8);
  std::cout << m << std::endl;
   Eigen::MatrixXf i;
   i = m.inverse();
  std::cout << std::endl << i << std::endl;
  IBVS object;
  object.disp_uv();
  object.disp_uv_row(2);
  std::cout << "\n now first row should be 2.2,3.4\n";
  object.update_uv_row(1,2.2,3.4);
  object.disp_uv();

  std::cout << "\n now should be 1,2,10,20,2,3,20,30\n";
  std::vector<cv::Point2f> newuv;
  newuv.push_back(cv::Point2f(1,2));
  newuv.push_back(cv::Point2f(10,20));
  newuv.push_back(cv::Point2f(2,3));
  newuv.push_back(cv::Point2f(20,30));
  object.update_uv(newuv);
  object.disp_uv();
std::cout << "\n now should be 10->3\n";
  std::vector<double> seconduv;
  for (int j=0;j<8;j++)
  {
      seconduv.push_back(10-j);
  }
  object.update_uv(seconduv);
object.disp_uv();

}

