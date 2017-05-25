#define _USE_MATH_DEFINES

#include "ibvs.h"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

velocity IBVS::calculate_vc()
{
    vc = Le_psinv * deltaS;
//    std::cout << " velocity is: " << vc(0,0) << " " << vc(1,0) << " " << vc(2,0) << " " << vc(3,0) << " " << vc(4,0) << " " << vc(5,0) << '\n';

    return vc;
}

void IBVS::calculate_deltaS()
{

    deltaS = ImagePts - desiredPts;

/*    std::cout << "deltaS:  ";
    for (int i = 0; i<deltaS.rows();i++)
    {
        std::cout << deltaS[i] << " ";
    }
    std::cout << std::endl << std::endl;
*/
}

void IBVS::display_params()
{
    std::cout << "Desired image : ";
    for (int i=0;i<desiredPts.rows();i++) { std::cout << desiredPts(i,0) << "  ";}
    std::cout << '\n';
    std::cout << "Image Points : ";
    for (int i=0;i<ImagePts.rows();i++) { std::cout << ImagePts(i,0)<< "  ";}
    std::cout << '\n';
    std::cout << "DeltaS : ";
    for (int i=0;i<deltaS.rows();i++) { std::cout << deltaS(i,0)<< "  ";}
    std::cout << '\n';
    std::cout << "z_est : " << z_est << '\n';

    std::cout << "v_desired : ";
    for (int i=0;i<vc.rows();i++) { std::cout << vc(i,0)<< "  ";}
    std::cout << '\n';
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

double IBVS::distance_calc(cv::Point2f pt1, cv::Point2f pt2)
{
    double result;

    result = sqrt((pt2.x-pt1.x)*(pt2.x-pt1.x)+(pt2.y-pt1.y)*(pt2.y-pt1.y));
    return result;
} // End distance_calc


void IBVS::rearrangeDesPts(std::vector<cv::Point2f> fourCorners)
{
    //std::cout << "IN REARRANGE DESPTS \n\n\n\n\n\n";
    int numpts = fourCorners.size();

    std::vector<cv::Point2f> desPt2f(4);

    desPt2f = uvToPoint2f(desiredPts);
    desPt2f.resize(4);

    double angleCurrent, angleDiff;

    angleCurrent = atan2((fourCorners[1].y - fourCorners[0].y),(fourCorners[1].x-fourCorners[0].x));
    std::cout << angleCurrent << "\n";
//Watch out, this goes increasing angles downwards.
    angleDiff = angleCurrent - angleDes;

    if(angleDiff >= 3*M_PI/4 || angleDiff < -3*M_PI/4)
    {
        desiredPts << 370,195,270,195,370,125,270,125;
        //std::cout <<'\n' << desiredPts << '\n';
    }
    else if(angleDiff >= M_PI/4)
    {
        std::cout << "value is between M_PI/4 and 3PI/4, rotating clockwise by 90 deg" << '\n';
        desiredPts << 370,195,270,125,370,125,270,195;
                std::cout <<'\n' << desiredPts << '\n';
    }
    else if(angleDiff <= -M_PI/4)
    {
        desiredPts << 370,125,370,195,270,125,270,195;
                std::cout <<'\n' << desiredPts << '\n';
    }
    else
    {
        std::cout << "no change" << '\n';
    }

/*
    // This is a vector of vectors (aka a matrix), 4x4 in most cases, and it will house the distance from each point to the other.
    double rtn_vec[numpts][numpts];

    std::cout << "\n rtn vec: \n";
    for(int i=0; i<numpts;i++)
    {
        //We use -1 from the old vector size because the mean is included in old vector
        for(int j = 0; j < numpts; j++)
        {
            // Calculate distance from each old vector (and the mean)
            rtn_vec[i][j] = distance_calc(desPt2f[j],fourCorners[i]);
            std::cout<< rtn_vec[i][j] << " ";
  //1          std::cout << "desPt2f: " << desPt2f[j] << " fourCorners: " << fourCorners[i] << "rtn_vec: " << rtn_vec[i][j] << "\n";
        }// end of internal nested for
        std::cout << '\n';
    } // end of external nested for

    double sum_dist_vec[4] = {0,0,0,0};
    int min_dist_ind = 0;




    // We now have a 4x4 matrix populated by the distances to nearby points.
    // We essentially want to select the lowest sum possible that still incorporates all distances.
    // The code below calculates the various ways of shifting the rectangle
    for (int i = 0; i<numpts; i++)
    {
        //This for loop basically adds up values along the diagonals of our matrix
        for (int j = 0; j<numpts; j++)
        {
            int k = within_bounds(j+i,4);
            sum_dist_vec[i] += rtn_vec[j][k];
  //1          std::cout << "j/k" << j << "/" << k << "   sum_dist_vec[i] " << sum_dist_vec[i] << " rtn_vec[j][k]" << rtn_vec[j][k] <<'\n';
        } // end inner for

  //1     std::cout << "sum dist vec " << i << ":  " << sum_dist_vec[i] << '\n';

        if (sum_dist_vec[i] < sum_dist_vec[min_dist_ind])
        {
            min_dist_ind = i;
        } // end if
    } // end outer for

    std::cout << "input fourCorners : " <<\
                 fourCorners[0] << "   " <<\
                 fourCorners[1] << "   " <<\
                 fourCorners[2] << "   " <<\
                 fourCorners[3] << "   " <<
                 std::endl;

    std::cout << "old desiredPts : " <<\
                 desiredPts[0] << "   " <<\
                 desiredPts[1] << "   " <<\
                 desiredPts[2] << "   " <<\
                 desiredPts[3] << "   " <<\
                 desiredPts[4] << "   " <<\
                 desiredPts[5] << "   " <<\
                 desiredPts[6] << "   " <<\
                 desiredPts[7] <<\
                 std::endl;

    uv oldDesPts;
    oldDesPts = desiredPts;

    // We now have an offset number. Now offset the
    for (int p = 0; p < desiredPts.rows(); p++)
    {
        //the 2*min_dist_ind is due to an 8 point shifting - see log from May 24th 2017
        int new_index = within_bounds(p+2*min_dist_ind,desiredPts.rows());
        desiredPts[new_index] = oldDesPts[p];
     //   std::cout << "shifting with offset number " << min_dist_ind << "  new_index: " << new_index << '\n';
     //   std::cout << "old des pt at that index: " << oldDesPts[p] << " new one: " << desiredPts[new_index] << '\n';
    }


    std::cout << "\nshift index: " << min_dist_ind << '\n';
    std::cout << "new desiredPts : " <<\
                 desiredPts[0] << "   " <<\
                 desiredPts[1] << "   " <<\
                 desiredPts[2] << "   " <<\
                 desiredPts[3] << "   " <<\
                 desiredPts[4] << "   " <<\
                 desiredPts[5] << "   " <<\
                 desiredPts[6] << "   " <<\
                 desiredPts[7] <<\
                 std::endl;
*/
} // end pvt_identify_pt

//This function is simply meant to help with the wraparound of the whole
int IBVS::within_bounds(int index, int totalind)
{
    int numpts;
    numpts = totalind;
    if(totalind= -1) {int numpts = desiredPts.rows();}
    if (index >= 2*numpts)
    {
        std::cout << "Index is twice what it should be";
    }
// If the index is greater than numpoints, that means that we want to be wrapping around the matrix
    else if (index >= numpts)
    {
        return (index-numpts);
    }
    else {return index;}
}

std::vector<cv::Point2f> IBVS::uvToPoint2f(uv input)
{
    int ptnum = input.rows()/2;
    std::vector<cv::Point2f> output(ptnum);

//    std::cout << "inside uvToPoint2f: \n" << "ptnum: " << ptnum<< " input: " << input << '\n';

    for(int i = 0; i<ptnum;i++)
    {
        output[i].x = input(2*i,0);
        output[i].y = input(2*i+1,0);
//        std::cout<< "in for loop: i = " << i << " output(x)/output(y): " << output[i].x << "/" << output[i].y;
    }

    return output;
}

uv IBVS::point2fToUv(std::vector<cv::Point2f> input)
{
    int ptnum = input.size();
    uv output;

    for(int i = 0; i<ptnum;i++)
    {
        output(2*i,0) = input[i].x;
        output(2*i+1,0) = input[i].y;
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
//    std::cout << '\n' << z_est << "\n";
}



void IBVS::MP_psinv_Le()
{
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(Le, Eigen::ComputeFullU | Eigen::ComputeFullV);

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

    Eigen::MatrixXf V, UT;
    V = svd.matrixV();
    UT = svd.matrixU().transpose();

    Le_psinv = V*DiagMat*UT;
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

void IBVS::manual_Le(std::vector<double> vecLe)
{
    LeMat newLe;

    for(int i=0;i<8;i++)
    {
        for(int j=0; j<6; j++)
        {
            newLe(i,j) = vecLe[i+j];
        }
    }
    Le = newLe;
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


IBVS::IBVS(double baseline, double focal_length, double camWidth, double camHeight)
{
    Pinv_tolerance = 0.1;
    correctDesiredPts = true;
    focal_lngth = focal_length;
    bsln = baseline;
    angleDes = 0; // sets the angle between points 0 and 1 desired
    camWdth = camWidth;
    camHght = camHeight;
    imageCenter = cv::Point2f(camWdth/2,camHght/2);
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

void IBVS::calc_desiredPts(cv::Point2f center, double offsetx, double offsety, double psi)
{
    Eigen::Matrix<double,2,4> vecMat;
    Eigen::Rotation2D<double> rotMat(90);
    //rotMat = Eigen::Rotation2D;
    vecMat << -offsetx,offsetx,-offsetx,offsetx,\
            -offsety,-offsety,offsety,offsety;

    std::cout << "vecMat: \n" << vecMat << "\n";
    std::vector<cv::Point2f> ctr2ptVec(4);
    ctr2ptVec[0] = cv::Point2f(-offsetx,-offsety);
    ctr2ptVec[1] = cv::Point2f(offsetx,-offsety);
    ctr2ptVec[2] = cv::Point2f(-offsetx,offsety);
    ctr2ptVec[3] = cv::Point2f(offsetx,offsety);
   // desiredPts =
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
}// end addPtsToImg
