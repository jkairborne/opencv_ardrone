#ifndef IBVS_H
#define IBVS_H

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<float, 8, 6> LeMat;
typedef Eigen::Matrix<float, 6, 8> LePlus;
typedef Eigen::Matrix<float, 8, 1> uv;


class IBVS {
    uv ImagePts;
    LeMat Le;
    LePlus Le_psinv, DiagMat;
    double old_z_hat;
    double Pinv_tolerance;
  public:
    void update_uv_row (int update_pair, double new_u, double new_v);
    void update_uv (std::vector<cv::Point2f> uv_new);
    void update_uv (std::vector<double> uv_new); //Overload for vector of doubles
    void update_tolerance(double newval);
    IBVS();
    void disp_uv_row(int n);
    void disp_uv();
    void update_Le_row(int, double);
    void update_Le(double);
    void display_Le();
    void MP_psinv_Le();
};

void IBVS::MP_psinv_Le()
{
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(Le, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    std::cout << "Sing Values: " << svd.singularValues() << std::endl;
//    std::cout << "U: " << svd.matrixU() << std::endl;
//    std::cout << "VT: " << svd.matrixV() << std::endl;

    int i = 0;
    for(i=0;i<6;i++)
    {
        // Here we have an If clause, simply on the chance that the values are too close to zero
        // This is a main reason for things being slow in calculating the jacobian, the singular values being near zero but not quite zero
        // leads to the values in the matrix blowing up.

        if(svd.singularValues()[i] > Pinv_tolerance || svd.singularValues()[i]<(-Pinv_tolerance))
        {
            DiagMat(i,i) = (1/svd.singularValues()[i]);
        }//endif
        else{ DiagMat(i,i) = 0;}//end else
    }//end for

    //chose not to use the diagonal matrix because it has issues with being displayed, and can't handle a rectangular matrix
    //   Eigen::DiagonalMatrix<float, 6> SingValMat(6);
    //   SingValMat.diagonal() = svd.singularValues();
//    std::cout << "\n" << DiagMat << "\n";

    Eigen::MatrixXf V, UT;
    V = svd.matrixV();
    UT = svd.matrixU().transpose();
//    std::cout << "\nV: " << V << "\n \n UT: " << UT;
//    std::cout << "\n\n" << DiagMat << "\n\n";

    Le_psinv = V*DiagMat*UT;

    std::cout << "Le_Psinv: \n" <<Le_psinv << '\n';

}

void IBVS::display_Le()
{
	std::cout << Le;
}


// n should be a number between 1 and 4, depending on the "blob" we want to keep track of.
void IBVS::update_Le_row(int n, double z_hat)
{
   int row_number1, row_number2;
   row_number1 = 2*(n-1);
   row_number2 = 2*n -1;
   Le(row_number1,0) = -1/z_hat;
   Le(row_number1,2) = ImagePts(row_number1,0) / z_hat;
   Le(row_number1,3) = ImagePts(row_number1,0) * ImagePts(row_number2,0);
   Le(row_number1,4) = - (1 + ImagePts(row_number1,0)*ImagePts(row_number1,0));
   Le(row_number1,5) = ImagePts(row_number2,0);
//Populate second row of that particular pair 
   Le(row_number2,1) = -1/z_hat;
   Le(row_number2,2) = ImagePts(row_number2,0) / z_hat;
   Le(row_number2,3) = 1 + ImagePts(row_number1,0)*ImagePts(row_number1,0);
   Le(row_number2,4) = -ImagePts(row_number1,0) * ImagePts(row_number2,0);
   Le(row_number2,5) = -ImagePts(row_number1,0);
}

void IBVS::update_Le(double z_hat)
{
	IBVS::update_Le_row(1,z_hat);
	IBVS::update_Le_row(2,z_hat);
	IBVS::update_Le_row(3,z_hat);
	IBVS::update_Le_row(4,z_hat);
}

void IBVS::update_uv (std::vector<cv::Point2f> uv_new)
{
    if(uv_new.size() != 4) {std::cout << "Your update_uv function call does not contain 4 pairs";}
    else
    {
       for(int j=0;j<uv_new.size();j++)
       {
           ImagePts(j*2,0) = uv_new[j].x;
           ImagePts(j*2+1,0) = uv_new[j].y;
       }
    }
}

void IBVS::update_uv (std::vector<double> uv_new)
{
    if(uv_new.size() != 8) {std::cout << "Your update_uv double function call does not contain 8 numbers";}
    else
    {
       for(int j=0;j<uv_new.size();j++)
       {
           ImagePts(j,0) = uv_new[j];
       }
    }
}



void IBVS::update_uv_row (int update_pair, double new_u, double new_v)
{
ImagePts(update_pair*2,0) = new_u;
ImagePts(update_pair*2+1,0) = new_v;
}

IBVS::IBVS()
{
    Pinv_tolerance = 0.1;

        ImagePts(0,0) = 1;
        ImagePts(1,0) = 2;
	ImagePts(2,0) = 3;
	ImagePts(3,0) = 4;
	ImagePts(4,0) = 5;
	ImagePts(5,0) = 6;
        ImagePts(6,0) = 7;
        ImagePts(7,0) = 8;
	Le <<	0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0;
	DiagMat <<	0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0;
	Le_psinv <<	0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0;

}

void IBVS::update_tolerance(double newval)
{
    Pinv_tolerance = newval;
}

void IBVS::disp_uv_row(int n)
{
std::cout << "\n Pair " << (n+1) << ": " <<"u = " << ImagePts(2*n,0) << ", v = " << ImagePts(2*n+1,0)<< '\n';;
}

void IBVS::disp_uv()
{
    for(int i=0;i<ImagePts.size();i++)
    {
        std::cout << ImagePts[i] << " ";
    }//end for
    std::cout << std::endl;
}// end disp_uv

#endif // IBVS_H
