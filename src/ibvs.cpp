#include "ibvs.h"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

velocity IBVS::calculate_vc()
{
    vc = Le_psinv * deltaS;
    std::cout << " velocity is: " << vc(0,0) << " " << vc(1,0) << " " << vc(2,0) << " " << vc(3,0) << " " << vc(4,0) << " " << vc(5,0) << '\n';

    return vc;
}

void IBVS::calculate_deltaS()
{

    deltaS = ImagePts - desiredPts;

    std::cout << "deltaS:  ";
    for (int i = 0; i<deltaS.rows();i++)
    {
        std::cout << deltaS[i] << " ";
    }
    std::cout << std::endl << std::endl;
}



std::vector<cv::Point2f> IBVS::eigenToPoint2fVector(Eigen::MatrixXd eigenMat)
{
    if (eigenMat.cols()!=1 || eigenMat.rows()%2 ) {std::cout << "eigenToPoint2fVector is receiving wrong dimension matrix - either not single column, or not even number of rows";}
    std::vector<cv::Point2f> output;

    int len = eigenMat.rows()/2;
    for(int i = 0; i< len; i++)
    {
        cv::Point2f newentry = cv::Point2f(eigenMat(2*i,0), eigenMat(2*i+1,0));

        output.push_back(newentry);
    }
    return output;
}

Eigen::MatrixXf IBVS::point2fVectorToEigenVec(std::vector<cv::Point2f> pnt2fVec)
{

//customize for uv etc        if (pnt2fVec.size() == 4)
        Eigen::MatrixXf output;
        for (int i = 0; i<pnt2fVec.size(); i++)
        {
            output(2*i) = pnt2fVec[i].x;
            output(2*i+1) = pnt2fVec[i].y;
        }

        return output;
}

std::vector<cv::Point2f> IBVS::getDesPtsPt2F()
{

    std::vector<cv::Point2f> output(desiredPts.rows()/2);
    for(int i = 0 ; i < desiredPts.rows()/2 ; i++)
    {
        output[i] = cv::Point2f(desiredPts(2*i),desiredPts(2*i+1));
    }
    return output;
}

void IBVS::update_z_est(std::vector<cv::Point2f> pts)
{
    double delta_x1, delta_y1, delta_x2, delta_y2, dist1, dist2, avg_dist;

    dist1 = cv::norm(pts[3]-pts[0]);
    dist2 = cv::norm(pts[2]-pts[1]);
    
    avg_dist = 0.5*(dist1+dist2);

    z_est = bsln*focal_lngth/avg_dist;
    std::cout << '\n' << z_est << "\n";
}



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

    std::cout << "Le_Psinv: \n" <<Le_psinv << '\n' << '\n';

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

void IBVS::update_Le()
{
    update_Le(z_est);
}

void IBVS::update_uv (std::vector<cv::Point2f> uv_new, bool updateDesired)
{
    if(uv_new.size() != 4) {std::cout << "Your update_uv function call does not contain 4 pairs";}
    else
    {
       if (updateDesired)
       {
           for(int j=0;j<uv_new.size();j++)
           {
               desiredPts(j*2,0) = uv_new[j].x;
               desiredPts(j*2+1,0) = uv_new[j].y;
           }
       }
       else
       {
            for(int j=0;j<uv_new.size();j++)
            {
                ImagePts(j*2,0) = uv_new[j].x;
                ImagePts(j*2+1,0) = uv_new[j].y;
            }
       }
    }
}

void IBVS::update_uv (std::vector<double> uv_new, bool updateDesired)
{
    if(uv_new.size() != 8) {std::cout << "Your update_uv double function call does not contain 8 numbers";}
    else
    {
        if(updateDesired)
        {
            for(int j=0;j<uv_new.size();j++)
            {
                desiredPts(j,0) = uv_new[j];
            }

        }
        else
        {
            for(int j=0;j<uv_new.size();j++)
            {
                ImagePts(j,0) = uv_new[j];
            }

        }
    }
}

void IBVS::update_uv_row (int update_pair, double new_u, double new_v, bool updateDesired)
{
    if(updateDesired)
    {
        desiredPts(update_pair*2,0) = new_u;
        desiredPts(update_pair*2+1,0) = new_v;
    }
    else
    {
        ImagePts(update_pair*2,0) = new_u;
        ImagePts(update_pair*2+1,0) = new_v;
    }
}


IBVS::IBVS(double baseline, double focal_length)
{
    Pinv_tolerance = 0.1;
    focal_lngth = focal_length;
    bsln = baseline;
    desiredPts << 270,125,370,125,270,195,370,195;

    ImagePts << 1,2,3,4,5,6,7,8;

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

void IBVS::update_desiredPts(std::vector<cv::Point2f> new_desPts)
{
    if(new_desPts.size() != 4) {std::cout << "Your update_desiredPts function call does not contain 4 pairs";}
    else
    {
       for(int j=0;j<new_desPts.size();j++)
       {
           ImagePts(j*2,0) = new_desPts[j].x;
           ImagePts(j*2+1,0) = new_desPts[j].y;
       }
    }
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
}

void IBVS::addPtsToImg(cv::Mat& img, std::vector<cv::Point2f> ptsToAdd, cv::Scalar color)
{
    for(int i = 0; i<ptsToAdd.size(); i++)
    {
        cv::circle(img, ptsToAdd[i], 1, color, -1 );
        std::string display_string;
        std::stringstream out;
        out << i;
        display_string = out.str();

        //Add numbering to the four points discovered.
        cv::putText( img, display_string, ptsToAdd[i], CV_FONT_HERSHEY_COMPLEX, 1,color, 1, 1);
    }
}// end disp_uv
